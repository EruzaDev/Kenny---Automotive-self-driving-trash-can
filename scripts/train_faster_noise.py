"""Train separate speed experiments and compare under matched simulated noise."""
import argparse
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed
import multiprocessing
import threading
import zipfile
from dataclasses import asdict, replace
import json
import os
import re
import shutil
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def review_initialized_checkpoint(run):
    """Use recorded guarded validation, never held-out evaluation, to select."""
    from kenny_rl.train import guarded_validation_score
    records = [json.loads(line) for line in (run/"validation.jsonl").read_text().splitlines()]
    candidates = [(run/"initialized.zip", records[0]), (run/"final.zip", records[-1])]
    selected, result = max(candidates, key=lambda item: guarded_validation_score(item[1]))
    shutil.copyfile(selected, run/"best_guarded.zip")
    (run/"checkpoint-selection.json").write_text(json.dumps({
        "criterion": "success, safety, navigation reward, interventions",
        "selected": selected.name, "validation_timesteps": result["timesteps"],
        "scores": {path.name: guarded_validation_score(record) for path, record in candidates},
        "held_out_data_used": False}, indent=2)+"\n")


def evaluate(checkpoint, environment, episodes, seed):
    import numpy as np
    import torch
    from stable_baselines3 import PPO
    from kenny_rl.config import load_config
    from kenny_rl.env import KennyEnv

    torch.set_num_threads(1)
    robot, _, training = load_config(checkpoint.parent / "config.json")
    env = KennyEnv(robot, environment)
    expected = json.loads((checkpoint.parent / "contract.json").read_text())
    if expected != json.loads(json.dumps(env.contract())):
        raise ValueError(f"Observation/control contract mismatch: {checkpoint}")
    model = PPO.load(checkpoint, device="cpu", custom_objects={
        "lr_schedule": lambda _: 0., "learning_rate": 0.,
        "clip_range": lambda _: training.get("clip_range", .2),
        "clip_range_vf": None})
    records = []
    speeds = []
    try:
        for index in range(episodes):
            obs, _ = env.reset(seed=seed + index)
            episode_speeds = []
            reward_sum = 0.
            while True:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                # Translation in the simulator includes the sampled slip factor.
                episode_speeds.append(abs(float(env.velocity[0] * env.slip[0])))
                reward_sum += reward
                if terminated or truncated:
                    break
            speeds.extend(episode_speeds)
            records.append({**info, "seed": seed + index, "reward": reward_sum,
                            "peak_speed_m_s": max(episode_speeds),
                            "mean_speed_m_s": float(np.mean(episode_speeds))})
            print(f"Evaluate {checkpoint.parent.name} {environment.split} "
                  f"{index + 1}/{episodes}: {info['event']}", flush=True)
    finally:
        env.close()
    steps = sum(r["steps"] for r in records)
    moving = [v for v in speeds if v > .01]
    return {
        "checkpoint": str(checkpoint.relative_to(ROOT)),
        "command_limit_m_s": robot.max_speed,
        "episodes": episodes, "first_seed": seed, "split": environment.split,
        "stage": environment.stage, "shield": True,
        "adaptive_speed_enabled": environment.adaptive_speed_enabled,
        "environment": asdict(environment),
        "success_rate": sum(r["is_success"] for r in records) / episodes,
        "collision_rate": sum(r["event"] == "collision" for r in records) / episodes,
        "person_collision_rate": sum(r["event"] == "collision" and r["collision_source"] == "person"
                                     for r in records) / episodes,
        "cliff_rate": sum(r["event"] == "cliff" for r in records) / episodes,
        "timeout_rate": sum(r["event"] == "timeout" for r in records) / episodes,
        "intervention_fraction": sum(r["interventions"] for r in records) / steps,
        "mean_steps": float(np.mean([r["steps"] for r in records])),
        "mean_success_steps": (float(np.mean([r["steps"] for r in records if r["is_success"]]))
                               if any(r["is_success"] for r in records) else None),
        "peak_speed_m_s": max(speeds),
        "mean_speed_m_s": float(np.mean(speeds)),
        "moving_p95_speed_m_s": float(np.percentile(moving, 95)) if moving else 0.,
        "records": records,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speeds", nargs="+", type=float, default=[.6, .8, 1.0])
    parser.add_argument("--pilot", action="store_true", help="8192-step diagnostic runs")
    parser.add_argument("--steps", type=int)
    parser.add_argument("--workers", type=int, default=1, choices=[1, 2, 3])
    parser.add_argument("--resume-experiment", action="store_true", help="Adopt completed/running jobs in this experiment")
    parser.add_argument("--initialize-baseline", action="store_true", help="Initialize policy weights from the baseline rather than cold-start cloning")
    parser.add_argument("--episodes", type=int, default=30)
    parser.add_argument("--seed", type=int, default=3)
    parser.add_argument("--test-seed", type=int, default=50000)
    parser.add_argument("--output", required=True, help="New directory inside runs/")
    parser.add_argument("--baseline", default="runs/server-mixed-21x21-reviewed/seed_3/best_guarded.zip")
    args = parser.parse_args()
    if args.episodes < 1 or (args.steps is not None and args.steps < 512):
        parser.error("episodes must be positive; steps must be at least 512")
    if not args.speeds or len(set(args.speeds)) != len(args.speeds) or any(s not in (.6, .8, 1.) for s in args.speeds):
        parser.error("Choose distinct speeds from 0.6, 0.8, 1.0")
    output = (ROOT / args.output).resolve()
    if not output.is_relative_to(ROOT / "runs") or (output.exists() and not args.resume_experiment):
        parser.error("Use a new output directory inside runs/")
    baseline = (ROOT / args.baseline).resolve()
    if not baseline.is_file():
        parser.error("Baseline checkpoint missing")
    from kenny_rl.config import load_config, validate_training
    template = json.loads((ROOT / "configs/faster_noise_060.json").read_text())
    for speed in args.speeds:
        data = json.loads(json.dumps(template))
        data["robot"]["max_speed"] = speed
        if args.pilot:
            data["training"].update(total_timesteps=8192, eval_freq=25000,
                                    checkpoint_freq=4096, eval_episodes=2,
                                    behavior_cloning_episodes=2)
        if args.steps is not None:
            data["training"]["total_timesteps"] = args.steps
        validate_training(data["training"])
    if args.resume_experiment:
        if args.steps is not None:
            parser.error("Resume preserves the recorded training budgets")
        manifest = json.loads((output / "experiment.json").read_text())
        jobs = manifest["jobs"]
        if (manifest["seed"] != args.seed or manifest["pilot"] != args.pilot or
                manifest["baseline"] != str(baseline) or
                manifest.get("initialize_baseline", False) != args.initialize_baseline or
                [job["speed_m_s"] for job in jobs] != args.speeds):
            parser.error("Resume arguments must match the recorded experiment")
        # Rebuild executable commands from trusted paths, not manifest commands.
        for job in jobs:
            tag = f"speed_{round(job['speed_m_s'] * 100):03d}"
            run = output / tag
            config_path = output / f"{tag}.json"
            job["run"] = str(run)
            job["command"] = [sys.executable, "-u", "-m", "kenny_rl.train", "--config",
                              str(config_path), "--run", str(run), "--seed", str(args.seed)]
            if args.initialize_baseline:
                job["command"] += ["--initialize-policy", str(baseline)]
    else:
        output.mkdir(parents=True)
        jobs = []
        for speed in args.speeds:
            tag = f"speed_{round(speed * 100):03d}"
            data = json.loads(json.dumps(template))
            data["robot"]["max_speed"] = speed
            if args.pilot:
                data["training"].update(total_timesteps=8192, eval_freq=25000,
                                        checkpoint_freq=4096, eval_episodes=2,
                                        behavior_cloning_episodes=2)
            if args.steps is not None:
                data["training"]["total_timesteps"] = args.steps
            config_path = output / f"{tag}.json"
            config_path.write_text(json.dumps(data, indent=2) + "\n")
            run = output / tag
            command = [sys.executable, "-u", "-m", "kenny_rl.train", "--config",
                       str(config_path), "--run", str(run), "--seed", str(args.seed)]
            if args.initialize_baseline:
                command += ["--initialize-policy", str(baseline)]
            jobs.append({"speed_m_s": speed, "command": command, "run": str(run),
                         "status": "pending"})
        manifest = {"pilot": args.pilot, "noise_ranges_are_provisional": True,
                    "initialize_baseline": args.initialize_baseline,
                    "baseline": str(baseline), "seed": args.seed, "jobs": jobs}
    manifest["workers"] = args.workers
    manifest["evaluation_episodes"] = args.episodes
    manifest["test_seed"] = args.test_seed
    manifest_lock = threading.Lock()
    def save_manifest():
        with manifest_lock:
            staging = output / "experiment.json.tmp"
            staging.write_text(json.dumps(manifest, indent=2) + "\n")
            staging.replace(output / "experiment.json")
    save_manifest()
    with tempfile.TemporaryDirectory(prefix="kenny-speed-mpl-") as mpl:
        process_env = {**os.environ, "MPLCONFIGDIR": mpl, "MPLBACKEND": "Agg",
                       "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1",
                       "OPENBLAS_NUM_THREADS": "1"}
        os.environ.update({key: process_env[key] for key in
                           ("MPLCONFIGDIR", "MPLBACKEND", "OMP_NUM_THREADS",
                            "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")})
        def train_job(job):
            run = Path(job["run"])
            if job["status"] == "trained":
                return
            if run.exists() and job["status"] == "training":
                print(f"Adopting active trainer {run.name}", flush=True)
                while not zipfile.is_zipfile(run / "final.zip"):
                    if (run / "interrupted.zip").is_file():
                        job["status"] = "failed"
                        save_manifest()
                        return
                    pattern = "^"+re.escape(" ".join(job["command"]))+"$"
                    active = subprocess.run(["pgrep", "-f", pattern],
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    if active.returncode != 0 and not zipfile.is_zipfile(run / "final.zip"):
                        job["status"] = "failed"
                        job["error"] = "Active trainer vanished before final.zip; resume a saved checkpoint explicitly"
                        save_manifest()
                        return
                    time.sleep(5)
                job["status"] = "trained"
                job["returncode"] = 0
                save_manifest()
                return
            if run.exists():
                raise ValueError(f"Partial run requires an explicit checkpoint resume: {run}")
            job["status"] = "training"
            save_manifest()
            print(f"Training {job['speed_m_s']:.2f} m/s -> {job['run']}", flush=True)
            started = time.monotonic()
            with (output / f"{run.name}.log").open("w") as log:
                result = subprocess.run(job["command"], cwd=ROOT, env=process_env,
                                        stdout=log, stderr=subprocess.STDOUT)
            job["seconds"] = time.monotonic() - started
            job["returncode"] = result.returncode
            job["status"] = "trained" if result.returncode == 0 else "failed"
            save_manifest()
            if result.returncode != 0:
                print(f"Training failed; inspect {run.name}.log", flush=True)
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            for future in as_completed([pool.submit(train_job, job) for job in jobs]):
                future.result()
        if args.initialize_baseline:
            for job in jobs:
                if job["status"] == "trained":
                    review_initialized_checkpoint(Path(job["run"]))
        # Compare each candidate and the baseline in identical worlds/noise.
        _, common, _ = load_config(output / f"speed_{round(args.speeds[0] * 100):03d}.json")
        candidates = [("baseline_030", baseline)] + [
            (Path(job["run"]).name, Path(job["run"]) / "best_guarded.zip")
            for job in jobs if job["status"] == "trained"]
        summaries = []
        evaluations = []
        for name, checkpoint in candidates:
            for split in ("test", "stress"):
                environment = replace(common, split=split, shield=True,
                                      train_unshielded_fraction=0.,
                                      recovery_enabled=False, route_recovery_enabled=False,
                                      marker_sweep_enabled=False,
                                      adaptive_speed_enabled=name != "baseline_030")
                if split == "stress":
                    environment = replace(environment, sensor_noise=.05, dropout=.05,
                                          marker_dropout=.35, motor_gain_range=(.8, 1.2),
                                          slip_range=(.85, 1.05), command_delay_max_steps=5,
                                          sensor_outage_probability=.02,
                                          sensor_outage_steps=(3, 10),
                                          motion_range_noise_per_mps=.04,
                                          motion_range_noise_per_radps=.02,
                                          motion_marker_dropout_per_mps=.25,
                                          motion_marker_dropout_per_radps=.15)
                evaluations.append((name, split, checkpoint, environment))
        # Separate processes bound memory and avoid sharing Torch/env state.
        context = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=min(args.workers, 2), mp_context=context) as pool:
            futures = {pool.submit(evaluate, checkpoint, environment, args.episodes, args.test_seed): (name, split)
                       for name, split, checkpoint, environment in evaluations}
            for future in as_completed(futures):
                name, split = futures[future]
                report = future.result()
                (output / f"{name}-{split}.json").write_text(json.dumps(report, indent=2) + "\n")
                summaries.append({k: v for k, v in report.items() if k != "records"})
                summaries.sort(key=lambda row: (row["command_limit_m_s"], row["split"]))
                (output / "comparison.json").write_text(json.dumps(summaries, indent=2) + "\n")
        manifest["evaluation_complete"] = True
        save_manifest()
    if any(job["status"] == "failed" for job in jobs):
        raise SystemExit(1)
    print(f"Finished. Comparison: {output / 'comparison.json'}", flush=True)


if __name__ == "__main__":
    main()
