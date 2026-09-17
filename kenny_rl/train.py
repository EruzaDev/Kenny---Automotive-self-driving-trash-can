"""Small-policy PPO with bounded worker count, reproducible evaluation and resume."""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import importlib.metadata
import hashlib
from functools import partial
from .config import load_config, serialize


def make_env(robot, config):
    from stable_baselines3.common.monitor import Monitor
    from .env import KennyEnv
    return Monitor(KennyEnv(robot, config))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="configs/laptop.json")
    p.add_argument("--run", required=True, help="New output directory; never silently overwrites")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--steps", type=int)
    p.add_argument("--stage", choices=["empty", "static", "mixed", "dynamic", "cliffs", "full"])
    p.add_argument("--resume", help="Checkpoint zip; write resumed run into a NEW directory")
    p.add_argument("--device", help="cpu or cuda:0; requires an appropriate PyTorch build")
    p.add_argument("--envs", type=int)
    args = p.parse_args()
    robot, config, training = load_config(args.config)
    if config.split != "train":
        p.error("Training requires the train split")
    if args.stage:
        config = replace(config, stage=args.stage)
    for arg, key in ((args.steps, "total_timesteps"), (args.device, "device"), (args.envs, "n_envs")):
        if arg is not None:
            training[key] = arg
    n_envs = training.get("n_envs", 1)
    if n_envs < 1 or training.get("total_timesteps", 1) < 1:
        p.error("Worker and transition counts must be positive")
    n_steps, batch = training.get("n_steps", 512), training.get("batch_size", 128)
    if n_steps < 1 or batch < 2 or n_steps*n_envs < 2 or (n_steps*n_envs) % batch:
        p.error("Rollout size must be >1 and divisible by batch_size")
    for key in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
        os.environ[key] = "1"
    os.environ["MPLBACKEND"] = "Agg"
    # Set thread limits before NumPy/Torch load, including in spawned workers.
    from .env import KennyEnv
    import torch
    from .runtime import check_device
    device = training.get("device", "cpu")
    try:
        device_info = check_device(device)
    except (ValueError, RuntimeError) as exc:
        p.error(str(exc))
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    torch.set_num_threads(training.get("torch_threads", 1))
    run = Path(args.run)
    if run.exists():
        p.error("Run directory exists; choose a new --run directory")
    contract = KennyEnv(robot, config).contract()
    if args.resume:
        source = Path(args.resume).resolve().parent
        if source.name == "checkpoints":
            source = source.parent
        old = json.loads((source / "contract.json").read_text())
        if old != json.loads(json.dumps(contract)):
            p.error("Resume observation/robot contract differs; cannot reuse the checkpoint")
    run.mkdir(parents=True)
    (run/"config.json").write_text(json.dumps(serialize(robot, config, training), indent=2))
    (run/"contract.json").write_text(json.dumps(contract, indent=2))
    import sys
    (run/"requirements.txt").write_text("\n".join(sorted(
        f"{d.metadata['Name']}=={d.version}" for d in importlib.metadata.distributions()))+"\n")
    (run/"metadata.json").write_text(json.dumps({"seed": args.seed, "resume": args.resume,
        "python": sys.version, "torch": torch.__version__, "schema": contract["version"],
        "hardware": device_info, "headless": True,
        "source_sha256": {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                          for f in Path(__file__).parent.glob("*.py")}}, indent=2))
    cls = SubprocVecEnv if training.get("vector_backend") == "subproc" and n_envs > 1 else DummyVecEnv
    kwargs = {"start_method": "spawn"} if cls is SubprocVecEnv else {}
    env = cls([partial(make_env, robot, config) for _ in range(n_envs)], **kwargs)
    env.seed(args.seed)
    ppo_options = {key: training.get(key, default) for key, default in
                   (("learning_rate", 3e-4), ("gamma", .99), ("gae_lambda", .95),
                    ("clip_range", .2), ("ent_coef", .005), ("target_kl", None))}
    if args.resume:
        model = PPO.load(args.resume, env=env, device=device, n_steps=n_steps, batch_size=batch,
                         n_epochs=training.get("n_epochs", 5), tensorboard_log=str(run/"tensorboard"),
                         **ppo_options)
        model.set_random_seed(args.seed)
    else:
        model = PPO("MlpPolicy", env, device=device, seed=args.seed, **ppo_options,
                    n_steps=n_steps, batch_size=batch, n_epochs=training.get("n_epochs", 5),
                    tensorboard_log=str(run/"tensorboard"),
                    policy_kwargs={"net_arch": {"pi": [128, 128], "vf": [128, 128]}}, verbose=1)
    from .evaluate import evaluate_model

    class Validation(BaseCallback):
        def __init__(self):
            super().__init__()
            self.last = 0
            self.best = None
        def _on_training_start(self):
            self.last = self.num_timesteps
        def _on_step(self):
            if self.num_timesteps-self.last < training.get("eval_freq", 25000):
                return True
            self.last = self.num_timesteps
            result = evaluate_model(self.model, robot, replace(config, split="validation"),
                                    training.get("eval_episodes", 5), seed=10000)
            result["timesteps"] = self.num_timesteps
            for key in ("success_rate", "collision_rate", "cliff_rate", "timeout_rate", "intervention_fraction"):
                self.logger.record(f"validation/{key}", result[key])
            with (run/"validation.jsonl").open("a") as f:
                f.write(json.dumps(result)+"\n")
            # Safety first, then success, then low intervention rate.
            score = (-result["collision_rate"]-result["cliff_rate"], result["success_rate"],
                     -result["intervention_fraction"])
            if self.best is None or score > self.best:
                self.best = score
                self.model.save(run/"best")
            print("Validation:", result, flush=True)
            return True
    checkpoint = CheckpointCallback(save_freq=max(1, training.get("checkpoint_freq", 25000)//n_envs),
                                    save_path=str(run/"checkpoints"), name_prefix="ppo")
    try:
        model.learn(total_timesteps=training.get("total_timesteps", 100000),
                    callback=[checkpoint, Validation()], reset_num_timesteps=not bool(args.resume))
        model.save(run/"final")
    except KeyboardInterrupt:
        model.save(run/"interrupted")
        print("Saved interrupted.zip; resume into a new run directory.")
    finally:
        env.close()


if __name__ == "__main__":
    main()
