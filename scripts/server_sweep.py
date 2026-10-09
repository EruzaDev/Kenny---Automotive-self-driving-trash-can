#!/usr/bin/env python3
"""Independent PPO seeds on bounded CPU/GPU slots; this is not distributed PPO."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from dataclasses import replace

from kenny_rl.config import load_config, validate_training


def worker_budget(devices, requested, configured, cpu_budget=None, learner_cpus=1):
    """Budget active slots, including each learner's configured Torch threads."""
    if not devices:
        raise ValueError("At least one device is required")
    if type(learner_cpus) is not int or learner_cpus < 1:
        raise ValueError("Learner CPU count must be a positive integer")
    for workers in (configured, requested):
        if workers is not None and (type(workers) is not int or workers < 1):
            raise ValueError("Worker count must be a positive integer")
    available = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else (os.cpu_count() or 1)
    for cap in (os.environ.get("SLURM_CPUS_PER_TASK"), cpu_budget):
        if cap is not None:
            if int(cap) < 1:
                raise ValueError("CPU budget must be positive")
            available = min(available, int(cap))
    limit = available // len(devices) - learner_cpus
    workers = min(configured, limit) if requested is None else requested
    if workers < 1 or workers > limit:
        raise ValueError(f"{available} allocated CPUs cannot support {workers} workers per run plus "
                         f"{len(devices)} learners ({learner_cpus} CPUs each); "
                         "request more CPUs or fewer concurrent jobs/workers")
    return workers, available


def training_slots(devices, jobs_per_device, seed_count):
    """Round-robin GPU slots, without budgeting slots that cannot have a job."""
    if not devices or len(set(devices)) != len(devices):
        raise ValueError("Devices must be nonempty and unique; use --jobs-per-device to share GPUs")
    if type(jobs_per_device) is not int or jobs_per_device < 1:
        raise ValueError("Jobs per device must be a positive integer")
    if seed_count < 1:
        raise ValueError("At least one seed is required")
    return (list(devices) * jobs_per_device)[:seed_count]


def take_job(pending, device):
    """Assign the next queued seed to whichever device slot became free."""
    index, template = pending.pop(0)
    command = list(template)
    command[command.index("--device") + 1] = device
    return index, command


def stop_process(process):
    """Let the learner save first, then bound cleanup of its worker group."""
    try:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=15)
        return
    except ProcessLookupError:
        return
    except subprocess.TimeoutExpired:
        pass
    for signum in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=5)
            return
        except subprocess.TimeoutExpired:
            continue


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="configs/server.json")
    p.add_argument("--output", required=True)
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2, 3])
    p.add_argument("--devices", nargs="+", default=[f"cuda:{i}" for i in range(4)],
                   help="Unique devices; indices respect CUDA_VISIBLE_DEVICES")
    p.add_argument("--jobs-per-device", type=int, default=1,
                   help="Concurrent independent learners per device (default: 1); shares GPU memory")
    p.add_argument("--stage", default="empty", choices=["empty", "static", "mixed", "dynamic", "cliffs", "full"])
    p.add_argument("--steps", type=int)
    p.add_argument("--envs", type=int, help="CPU workers per run; default caps config by CPU allocation")
    p.add_argument("--cpu-budget", type=int, help="Total CPUs allocated to this launcher, including learners")
    p.add_argument("--resume-from", type=Path, help="Previous sweep directory containing seed_N runs")
    p.add_argument("--resume-checkpoint", default="best.zip",
                   choices=["best.zip", "best_guarded.zip", "final.zip", "interrupted.zip"])
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()
    if len(set(args.seeds)) != len(args.seeds):
        p.error("Seeds must be unique")
    try:
        slots = training_slots(args.devices, args.jobs_per_device, len(args.seeds))
    except ValueError as exc:
        p.error(str(exc))
    if args.steps is not None and args.steps < 1:
        p.error("Steps must be positive")
    robot, config, training = load_config(args.config)
    if config.split != "train":
        p.error("Training requires the train split")
    config = replace(config, stage=args.stage)
    try:
        workers, cpus = worker_budget(slots, args.envs, training.get("n_envs", 1), args.cpu_budget,
                                     training.get("torch_threads", 1))
        effective = {**training, "n_envs": workers}
        if args.steps is not None:
            effective["total_timesteps"] = args.steps
        for device in args.devices:
            validate_training({**effective, "device": device})
    except ValueError as exc:
        p.error(str(exc))
    root = Path(args.output)
    if root.exists():
        p.error("Output directory already exists")
    commands = []
    for i, seed in enumerate(args.seeds):
        command = [sys.executable, "-u", "-m", "kenny_rl.train", "--config", args.config,
                   "--run", str(root/f"seed_{seed}"), "--seed", str(seed), "--stage", args.stage,
                   "--device", slots[i % len(slots)], "--envs", str(workers)]
        if args.steps:
            command += ["--steps", str(args.steps)]
        if args.resume_from:
            source = args.resume_from/f"seed_{seed}"
            for name in (args.resume_checkpoint, "config.json", "contract.json"):
                if not (source/name).is_file():
                    p.error(f"Missing resume artifact: {source/name}")
            from kenny_rl.env import KennyEnv
            old = json.loads((source/"contract.json").read_text())
            if old != json.loads(json.dumps(KennyEnv(robot, config).contract())):
                p.error(f"Resume observation/robot contract differs: {source}")
            command += ["--resume", str(source/args.resume_checkpoint)]
        commands.append(command)
    print(f"Headless: {len(slots)} concurrent slots, {workers} workers/run, {cpus} allocated CPUs", flush=True)
    if args.jobs_per_device > 1:
        print("Shared devices: benchmark throughput and GPU/RAM usage before a long sweep.", flush=True)
    if args.dry_run:
        import shlex
        for command in commands:
            print(shlex.join(command))
        return
    # Preflight before launching any long jobs. Do not clear scheduler visibility.
    from kenny_rl.runtime import check_device
    try:
        hardware = [check_device(device) for device in dict.fromkeys(slots)]
    except (ValueError, RuntimeError) as exc:
        p.error(str(exc))
    child_env = os.environ.copy()
    child_env.update({"MPLBACKEND": "Agg", "PYTHONUNBUFFERED": "1",
                      "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1"})
    root.mkdir(parents=True)
    manifest = {"hardware": hardware, "headless": True, "cpus": cpus,
                "workers_per_run": workers, "jobs_per_device": args.jobs_per_device,
                "slot_devices": slots, "commands": commands, "launches": []}
    (root/"sweep.json").write_text(json.dumps(manifest, indent=2))
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    active, pending, failed = [], list(enumerate(commands)), []
    try:
        while pending or active:
            busy = {slot for _, _, _, slot in active}
            for slot, device in enumerate(slots):
                if slot in busy or not pending:
                    continue
                i, command = take_job(pending, device)
                log = (root/f"seed_{args.seeds[i]}.log").open("w")
                try:
                    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                               start_new_session=True, env=child_env)
                except BaseException:
                    log.close()
                    raise
                active.append((process, log, args.seeds[i], slot))
                manifest["launches"].append({"seed": args.seeds[i], "device": device,
                                              "slot": slot, "command": command})
                (root/"sweep.json").write_text(json.dumps(manifest, indent=2))
                print(f"Started seed {args.seeds[i]} on {device} (slot {slot})", flush=True)
            for item in active[:]:
                process, log, seed, slot = item
                if process.poll() is not None:
                    log.close()
                    active.remove(item)
                    if process.returncode:
                        failed.append(seed)
                    print(f"Seed {seed} exited with {process.returncode}", flush=True)
            time.sleep(.2)
    finally:
        for process, log, _, _ in active:
            if process.poll() is None:
                stop_process(process)
            log.close()
    if failed:
        raise SystemExit(f"Failed seeds: {failed}; see per-run logs")


if __name__ == "__main__":
    main()
