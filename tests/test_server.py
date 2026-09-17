import os
import subprocess
import sys

import pytest

from scripts.server_sweep import worker_budget


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
    assert train["target_kl"] == .02
