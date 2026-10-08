import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from kenny_rl.config import EnvConfig, RobotConfig, load_config, validate_training


@pytest.mark.parametrize("settings", [
    {"n_envs": 1.5}, {"n_envs": True}, {"n_steps": 0}, {"n_epochs": 0},
    {"total_timesteps": 0}, {"eval_episodes": 0}, {"eval_freq": 0},
    {"checkpoint_freq": -1}, {"learning_rate": float("nan")},
    {"learning_rate": 0}, {"gamma": 1.1}, {"gae_lambda": -1},
    {"ent_coef": float("inf")}, {"clip_range": 0}, {"target_kl": -1},
    {"behavior_cloning_episodes": -1}, {"behavior_cloning_epochs": 0},
    {"behavior_cloning_batch_size": 0}, {"behavior_cloning_log_std": 50},
    {"dual_validation": "false"}, {"vector_backend": "typo"},
    {"device": "auto"}, {"batch_size": 3}, {"learning_rte": .001},
])
def test_invalid_training_settings_fail_preflight(settings):
    with pytest.raises(ValueError):
        validate_training(settings)


def test_included_profiles_pass_preflight():
    for path in Path("configs").glob("*.json"):
        _, _, training = load_config(path)
        validate_training(training)


@pytest.mark.parametrize("value", [float("nan"), float("inf")])
def test_nonfinite_physics_and_environment_are_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        RobotConfig(acceleration=value)
    with pytest.raises(ValueError, match="finite"):
        EnvConfig(sensor_noise=value)


def test_final_updated_policy_is_validated_and_selected(tmp_path, monkeypatch):
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
    from kenny_rl import evaluate
    from kenny_rl.train import make_env, train_model

    updates = []
    modes = []
    def score(model, robot, config, episodes, seed):
        updates.append(model._n_updates)
        modes.append(config.shield)
        return {"success_rate": float(model._n_updates > 0), "collision_rate": 0.,
                "cliff_rate": 0., "timeout_rate": 0., "intervention_fraction": 0.,
                "no_route_fraction": 0., "intervention_reason_fractions": {},
                "unsafe_command_fraction": 0., "mean_reward": 0.}
    monkeypatch.setattr(evaluate, "evaluate_model", score)
    robot = RobotConfig()
    config = EnvConfig(stage="empty", max_steps=4)
    training = {"n_steps": 8, "batch_size": 8, "n_epochs": 1, "total_timesteps": 8,
                "eval_freq": 8, "eval_episodes": 1, "dual_validation": True}
    env = DummyVecEnv([lambda: make_env(robot, config)])
    try:
        train_model(robot, config, training, SimpleNamespace(seed=0, resume=None), tmp_path, env, "cpu")
    finally:
        env.close()
    # Initial/step validations precede the update; training-end sees the update.
    assert updates == [0, 0, 0, 0, 1, 1]
    assert modes == [True, False] * 3
    for name in ("final", "best", "best_guarded"):
        assert PPO.load(tmp_path/f"{name}.zip", device="cpu")._n_updates == 1
    for name in ("validation", "validation_unshielded"):
        results = [json.loads(line) for line in (tmp_path/f"{name}.jsonl").read_text().splitlines()]
        assert results[-1]["success_rate"] == 1.
        assert results[-1]["timesteps"] == 8


@pytest.mark.parametrize("failure", ["setup", "cloning"])
def test_training_closes_workers_when_setup_or_cloning_fails(tmp_path, monkeypatch, failure):
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
    from kenny_rl import bootstrap, train

    config_path = tmp_path/"input.json"
    config_path.write_text(json.dumps({"training": {"behavior_cloning_episodes": 1}}))
    closed = []
    original_close = DummyVecEnv.close
    def close(self):
        closed.append(True)
        original_close(self)
    def fail(*args, **kwargs):
        raise RuntimeError("injected failure")
    monkeypatch.setattr(DummyVecEnv, "close", close)
    if failure == "setup":
        monkeypatch.setattr(PPO, "__init__", fail)
    else:
        monkeypatch.setattr(bootstrap, "collect_demonstrations", fail)
    monkeypatch.setattr(sys, "argv", ["train", "--config", str(config_path), "--run", str(tmp_path/"run")])
    old_handler = signal.getsignal(signal.SIGTERM)
    with pytest.raises(RuntimeError, match="injected failure"):
        train.main()
    assert closed == [True]
    assert signal.getsignal(signal.SIGTERM) == old_handler


def test_invalid_config_creates_no_run_directory(tmp_path):
    config = tmp_path/"bad.json"
    config.write_text(json.dumps({"training": {"n_epochs": 0}}))
    run = tmp_path/"run"
    result = subprocess.run([sys.executable, "-m", "kenny_rl.train", "--config", str(config),
                             "--run", str(run)], capture_output=True, text=True)
    assert result.returncode == 2
    assert "n_epochs" in result.stderr
    assert not run.exists()


def test_cloning_uses_host_dataset_and_improves_policy(monkeypatch):
    import torch
    from stable_baselines3 import PPO
    from kenny_rl.bootstrap import clone_policy
    from kenny_rl.env import KennyEnv

    torch.set_num_threads(1)
    env = KennyEnv(config=EnvConfig(stage="empty"))
    model = PPO("MlpPolicy", env, n_steps=8, batch_size=8, device="cpu", seed=0)
    observations = np.zeros((32, env.observation_space.shape[0]), dtype=np.float32)
    actions = np.full((32, 2), .5, dtype=np.float32)
    conversions = []
    original = torch.as_tensor
    def as_tensor(value, *args, **kwargs):
        conversions.append((np.shape(value), kwargs.get("device")))
        return original(value, *args, **kwargs)
    monkeypatch.setattr(torch, "as_tensor", as_tensor)
    try:
        losses = clone_policy(model, observations, actions, 4, 8, .01, -2., 0)
        assert losses[-1] < losses[0]
        assert conversions[:2] == [(observations.shape, None), (actions.shape, None)]
        assert torch.all(model.policy.log_std == -2.)
        assert not model.policy.training
    finally:
        model.get_env().close()


def test_sweep_shutdown_has_bounded_escalation(monkeypatch):
    from scripts import server_sweep
    sent, waits = [], []
    class StuckProcess:
        pid = 123
        def send_signal(self, signum):
            sent.append((self.pid, signum))
        def wait(self, timeout):
            waits.append(timeout)
            if len(waits) < 3:
                raise subprocess.TimeoutExpired("training", timeout)
    monkeypatch.setattr(server_sweep.os, "killpg", lambda pid, sig: sent.append((pid, sig)))
    server_sweep.stop_process(StuckProcess())
    assert waits == [15, 5, 5]
    assert sent == [(123, signal.SIGINT), (123, signal.SIGTERM), (123, signal.SIGKILL)]


def test_nonfinite_observations_abort_training(tmp_path, monkeypatch):
    from stable_baselines3.common.vec_env import DummyVecEnv
    from kenny_rl import train
    original_reset = DummyVecEnv.reset
    def reset(self):
        return np.full_like(original_reset(self), np.nan)
    monkeypatch.setattr(DummyVecEnv, "reset", reset)
    run = tmp_path/"nan-run"
    monkeypatch.setattr(sys, "argv", ["train", "--config", "configs/smoke.json", "--run", str(run)])
    with pytest.raises(ValueError, match="nan.*observations"):
        train.main()
    assert not (run/"final.zip").exists()


def test_spawned_training_resume_and_sigterm(tmp_path):
    from stable_baselines3 import PPO
    config = tmp_path/"config.json"
    config.write_text(json.dumps({"environment": {"stage": "empty", "max_steps": 2},
        "training": {"n_envs": 2, "vector_backend": "subproc", "n_steps": 8,
                     "batch_size": 16, "n_epochs": 1, "total_timesteps": 16,
                     "eval_freq": 16, "eval_episodes": 1, "checkpoint_freq": 16}}))
    command = [sys.executable, "-u", "-m", "kenny_rl.train", "--config", str(config)]
    initial, resumed, interrupted = [tmp_path/name for name in ("initial", "resumed", "interrupted")]
    for run, source in ((initial, None), (resumed, initial/"final.zip")):
        args = [*command, "--run", str(run)]
        if source:
            args += ["--resume", str(source)]
        result = subprocess.run(args, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stderr
        assert (run/"checkpoints").is_dir()
    assert PPO.load(resumed/"final.zip", device="cpu").num_timesteps == 32
    latest = json.loads((resumed/"validation.jsonl").read_text().splitlines()[-1])
    assert latest["timesteps"] == 32

    with (tmp_path/"interrupted.log").open("w+") as log:
        process = subprocess.Popen([*command, "--run", str(interrupted), "--steps", "1000000",
                                    "--resume", str(resumed/"final.zip")],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while not (interrupted/"validation.jsonl").exists() and process.poll() is None:
                if time.monotonic() > deadline:
                    pytest.fail("Training never reached its initial validation")
                time.sleep(.02)
            assert process.poll() is None
            process.send_signal(signal.SIGTERM)
            assert process.wait(timeout=10) == 130
            assert (interrupted/"interrupted.zip").is_file()
            assert not (interrupted/"final.zip").exists()
            restored = PPO.load(interrupted/"interrupted.zip", device="cpu")
            assert restored.num_timesteps >= 32
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
