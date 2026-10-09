import os
import subprocess
import sys

import pytest

from scripts.server_sweep import take_job, training_slots, worker_budget


def test_shared_gpu_slots_and_cpu_budget(monkeypatch):
    monkeypatch.setattr(os, "sched_getaffinity", lambda _: set(range(64)))
    monkeypatch.delenv("SLURM_CPUS_PER_TASK", raising=False)
    devices = [f"cuda:{i}" for i in range(4)]
    slots = training_slots(devices, 2, 12)
    assert slots == devices * 2
    assert worker_budget(slots, None, 7, 32) == (3, 32)
    assert worker_budget(slots, None, 7, 32, learner_cpus=2) == (2, 32)
    assert training_slots(devices, 2, 2) == devices[:2]
    with pytest.raises(ValueError, match="allocated CPUs"):
        worker_budget(slots, 7, 7, 32)
    with pytest.raises(ValueError):
        training_slots(devices, 0, 8)
    with pytest.raises(ValueError, match="unique"):
        training_slots(["cuda:0", "cuda:0"], 1, 8)
    assert len(training_slots([f"cuda:{i}" for i in range(8)], 1, 8)) == 8


def test_next_job_uses_free_slot_not_static_seed_assignment():
    template = ["python", "--seed", "8", "--device", "cuda:0"]
    pending = [(8, template), (9, template)]
    index, command = take_job(pending, "cuda:3")
    assert index == 8 and command[-1] == "cuda:3"
    assert template[-1] == "cuda:0"
    assert len(pending) == 1


def test_shared_slots_launch_all_jobs_and_close_logs(monkeypatch, tmp_path):
    from scripts import server_sweep
    from kenny_rl import runtime
    monkeypatch.setattr(os, "sched_getaffinity", lambda _: set(range(32)))
    monkeypatch.delenv("SLURM_CPUS_PER_TASK", raising=False)
    monkeypatch.setattr(runtime, "check_device", lambda device: {"device": device})
    monkeypatch.setattr(server_sweep.signal, "signal", lambda *args: None)
    monkeypatch.setattr(server_sweep.time, "sleep", lambda _: None)
    processes = []

    class Process:
        def __init__(self, command, stdout, **kwargs):
            self.command, self.log = command, stdout
            self.returncode = None
            self.polls = 0
            processes.append(self)
            assert sum(p.returncode is None for p in processes) <= 8
            assert sum(p.returncode is None and p.command[p.command.index("--device")+1] ==
                       command[command.index("--device")+1] for p in processes) <= 2

        def poll(self):
            self.polls += 1
            if self.polls >= 2:
                self.returncode = 0
            return self.returncode

    monkeypatch.setattr(server_sweep.subprocess, "Popen", Process)
    root = tmp_path/"sweep"
    monkeypatch.setattr(sys, "argv", ["server_sweep", "--output", str(root), "--jobs-per-device", "2",
                                     "--cpu-budget", "32", "--seeds", *map(str, range(12))])
    server_sweep.main()
    assert len(processes) == 12
    assert all(p.log.closed for p in processes)
    import json
    manifest = json.loads((root/"sweep.json").read_text())
    assert manifest["workers_per_run"] == 3
    assert len(manifest["launches"]) == 12
    assert {row["seed"] for row in manifest["launches"]} == set(range(12))


def test_launch_failure_closes_log_and_stops_active_jobs(monkeypatch, tmp_path):
    from scripts import server_sweep
    from kenny_rl import runtime
    monkeypatch.setattr(os, "sched_getaffinity", lambda _: set(range(32)))
    monkeypatch.delenv("SLURM_CPUS_PER_TASK", raising=False)
    monkeypatch.setattr(runtime, "check_device", lambda device: {})
    monkeypatch.setattr(server_sweep.signal, "signal", lambda *args: None)
    logs, stopped = [], []

    class Process:
        def poll(self):
            return None

    process = Process()

    def launch(command, stdout, **kwargs):
        logs.append(stdout)
        if len(logs) == 2:
            raise OSError("launch failed")
        return process

    monkeypatch.setattr(server_sweep.subprocess, "Popen", launch)
    monkeypatch.setattr(server_sweep, "stop_process", stopped.append)
    monkeypatch.setattr(sys, "argv", ["server_sweep", "--output", str(tmp_path/"sweep")])
    with pytest.raises(OSError, match="launch failed"):
        server_sweep.main()
    assert stopped == [process]
    assert all(log.closed for log in logs)


def test_worker_budget_respects_affinity_scheduler_and_user(monkeypatch):
    monkeypatch.setattr(os, "sched_getaffinity", lambda _: set(range(64)))
    monkeypatch.setenv("SLURM_CPUS_PER_TASK", "36")
    devices = [f"cuda:{i}" for i in range(4)]
    assert worker_budget(devices, None, 8) == (8, 36)
    assert worker_budget(devices, None, 8, 20) == (4, 20)
    with pytest.raises(ValueError, match="allocated CPUs"):
        worker_budget(devices, 8, 8, 20)
    with pytest.raises(ValueError):
        worker_budget(devices, None, 8, 4)


def test_four_gpu_dry_run_has_unique_seeds_and_headless_workers(tmp_path):
    env = os.environ.copy()
    env.pop("SLURM_CPUS_PER_TASK", None)
    if hasattr(os, "sched_getaffinity") and len(os.sched_getaffinity(0)) < 8:
        pytest.skip("Four concurrent learners/workers require eight CPU slots")
    result = subprocess.run([sys.executable, "scripts/server_sweep.py", "--output", str(tmp_path/"sweep"),
                             "--cpu-budget", "8", "--dry-run"], env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    for i in range(4):
        assert f"--device cuda:{i} --envs 1" in result.stdout
        assert f"--seed {i} --stage empty" in result.stdout
    assert not (tmp_path/"sweep").exists()


def test_cuda_unavailable_is_rejected(monkeypatch):
    torch = pytest.importorskip("torch")
    from kenny_rl.runtime import check_device
    monkeypatch.setattr(torch.cuda, "is_available", lambda: False)
    with pytest.raises(ValueError, match="unavailable"):
        check_device("cuda:0")


def test_server_config_rollout_and_disturbances():
    from kenny_rl.config import load_config
    _, env, train = load_config("configs/server.json")
    assert env.domain_randomization and env.shield
    assert env.sensor_outage_probability > 0
    assert train["n_envs"] * train["n_steps"] % train["batch_size"] == 0
    assert train["target_kl"] == .005
    assert train["learning_rate"] == 1e-5
    assert train["clip_range"] == .05


def test_bootstrap_removes_disturbances_but_preserves_contract():
    from kenny_rl.config import load_config
    from kenny_rl.env import KennyEnv
    robot, bootstrap, train = load_config("configs/server_bootstrap.json")
    _, robust, _ = load_config("configs/server.json")
    assert not bootstrap.domain_randomization
    assert bootstrap.sensor_noise == bootstrap.dropout == bootstrap.marker_dropout == 0
    assert bootstrap.sensor_outage_probability == 0
    assert train["total_timesteps"] == 250000
    assert train["behavior_cloning_episodes"] == 200
    assert KennyEnv(robot, bootstrap).contract() == KennyEnv(robot, robust).contract()


def test_bootstrap_collects_only_successful_demonstrations():
    from kenny_rl.bootstrap import collect_demonstrations
    from kenny_rl.config import load_config
    robot, config, _ = load_config("configs/server_bootstrap.json")
    observations, actions = collect_demonstrations(robot, config, 2, seed=0)
    assert len(observations) == len(actions) > 0
    assert observations.shape[1] > actions.shape[1] == 2
