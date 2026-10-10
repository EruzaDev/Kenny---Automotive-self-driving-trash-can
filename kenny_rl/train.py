"""Small-policy PPO with bounded worker count, reproducible evaluation and resume."""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import importlib.metadata
import hashlib
import signal
from functools import partial
from .config import load_config, serialize, validate_training


def make_env(robot, config):
    from stable_baselines3.common.monitor import Monitor
    from .env import KennyEnv
    return Monitor(KennyEnv(robot, config))


def validate_speed_transfer(old, new):
    """Only permit speed-cap/controller changes for policy initialization."""
    old, new = json.loads(json.dumps(old)), json.loads(json.dumps(new))
    for contract in (old, new):
        contract.pop("adaptive_speed_controller", None)
        contract["robot"].pop("max_speed")
    if old != new:
        raise ValueError("Policy initialization requires matching sensors, observations and robot dynamics")


def guarded_validation_score(result):
    # If both candidates time out safely, prefer navigation reward before
    # fewer interventions. An idle policy otherwise wins by requesting less.
    return (result["success_rate"], -result["collision_rate"]-result["cliff_rate"],
            result["mean_reward"], -result["intervention_fraction"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="configs/laptop.json")
    p.add_argument("--run", required=True, help="New output directory; never silently overwrites")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--steps", type=int)
    p.add_argument("--stage", choices=["empty", "static", "mixed", "dynamic", "cliffs", "full"])
    p.add_argument("--resume", help="Checkpoint zip; write resumed run into a NEW directory")
    p.add_argument("--initialize-policy", help="Copy compatible policy weights for a new speed experiment; resets PPO optimizer and counters")
    p.add_argument("--device", help="cpu or cuda:0; requires an appropriate PyTorch build")
    p.add_argument("--envs", type=int)
    args = p.parse_args()
    if args.resume and args.initialize_policy:
        p.error("Choose resume or policy initialization")
    robot, config, training = load_config(args.config)
    if config.split != "train":
        p.error("Training requires the train split")
    if args.stage:
        config = replace(config, stage=args.stage)
    for arg, key in ((args.steps, "total_timesteps"), (args.device, "device"), (args.envs, "n_envs")):
        if arg is not None:
            training[key] = arg
    try:
        validate_training(training)
    except ValueError as exc:
        p.error(str(exc))
    n_envs = training.get("n_envs", 1)
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
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecCheckNan
    torch.set_num_threads(training.get("torch_threads", 1))
    run = Path(args.run)
    if run.exists():
        p.error("Run directory exists; choose a new --run directory")
    contract = KennyEnv(robot, config).contract()
    if args.resume:
        if not Path(args.resume).is_file():
            p.error(f"Resume checkpoint does not exist: {args.resume}")
        source = Path(args.resume).resolve().parent
        if source.name == "checkpoints":
            source = source.parent
        old = json.loads((source / "contract.json").read_text())
        if old != json.loads(json.dumps(contract)):
            p.error("Resume observation/robot contract differs; cannot reuse the checkpoint")
    if args.initialize_policy:
        source = Path(args.initialize_policy).resolve().parent
        if source.name == "checkpoints":
            source = source.parent
        try:
            if not Path(args.initialize_policy).is_file():
                raise ValueError("Initialization checkpoint does not exist")
            validate_speed_transfer(json.loads((source/"contract.json").read_text()), contract)
        except (ValueError, OSError) as exc:
            p.error(str(exc))
    run.mkdir(parents=True)
    (run/"config.json").write_text(json.dumps(serialize(robot, config, training), indent=2))
    (run/"contract.json").write_text(json.dumps(contract, indent=2))
    import sys
    (run/"requirements.txt").write_text("\n".join(sorted(
        f"{d.metadata['Name']}=={d.version}" for d in importlib.metadata.distributions()))+"\n")
    (run/"metadata.json").write_text(json.dumps({"seed": args.seed, "resume": args.resume,
        "initialize_policy": args.initialize_policy,
        "python": sys.version, "torch": torch.__version__, "schema": contract["version"],
        "hardware": device_info, "headless": True,
        "source_sha256": {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                          for f in Path(__file__).parent.glob("*.py")}}, indent=2))
    cls = SubprocVecEnv if training.get("vector_backend") == "subproc" and n_envs > 1 else DummyVecEnv
    kwargs = {"start_method": "spawn"} if cls is SubprocVecEnv else {}
    env = cls([partial(make_env, robot, config) for _ in range(n_envs)], **kwargs)
    def terminate(signum, frame):
        raise KeyboardInterrupt
    previous_handler = signal.signal(signal.SIGTERM, terminate)
    try:
        env = VecCheckNan(env, raise_exception=True)
        train_model(robot, config, training, args, run, env, device)
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
        env.close()


def train_model(robot, config, training, args, run, env, device):
    """Fit using an environment whose owner closes it even if setup fails."""
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    n_envs = training.get("n_envs", 1)
    n_steps, batch = training.get("n_steps", 512), training.get("batch_size", 128)
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
        if getattr(args, "initialize_policy", None):
            source_model = PPO.load(args.initialize_policy, device=device, custom_objects={
                "lr_schedule": lambda _: 0., "learning_rate": 0.,
                "clip_range": lambda _: .2, "clip_range_vf": None})
            model.policy.load_state_dict(source_model.policy.state_dict(), strict=True)
            del source_model
            model.save(run/"initialized")
            print("Initialized policy weights; PPO optimizer and step counter are new", flush=True)
    cloning_episodes = training.get("behavior_cloning_episodes", 0)
    clone_now = cloning_episodes and not getattr(args, "initialize_policy", None) and (not args.resume or training.get("behavior_cloning_on_resume", False))
    if clone_now:
        from .bootstrap import collect_demonstrations, clone_policy
        demonstration_config = config
        if training.get("behavior_cloning_stage") is not None:
            demonstration_config = replace(demonstration_config, stage=training["behavior_cloning_stage"])
        if training.get("behavior_cloning_unshielded", False):
            demonstration_config = replace(demonstration_config, shield=False, train_unshielded_fraction=0.)
        observations, actions = collect_demonstrations(robot, demonstration_config,
                                                        cloning_episodes, args.seed*100000)
        losses = clone_policy(model, observations, actions,
                              training.get("behavior_cloning_epochs", 10),
                              training.get("behavior_cloning_batch_size", 512),
                              training.get("behavior_cloning_learning_rate", 3e-4),
                              training.get("behavior_cloning_log_std", -1.), args.seed)
        model.save(run/"distilled")
        print(f"Behavior cloning: {len(observations)} samples, "
              f"loss {losses[0]:.6f} -> {losses[-1]:.6f}; saved distilled.zip", flush=True)
    from .evaluate import evaluate_model

    class Validation(BaseCallback):
        def __init__(self):
            super().__init__()
            self.last = 0
            self.best = None
            self.best_guarded = None
        def _on_training_start(self):
            self.last = self.num_timesteps
            # Score the resumed or cloned policy before PPO can change it. This
            # also guarantees that every completed run has a best.zip.
            self._evaluate()
        def _evaluate(self):
            result = evaluate_model(self.model, robot, replace(config, split="validation",
                                    shield=True if training.get("dual_validation", False) else config.shield),
                                    training.get("eval_episodes", 5), seed=10000)
            result["timesteps"] = self.num_timesteps
            for key in ("success_rate", "collision_rate", "cliff_rate", "timeout_rate", "intervention_fraction"):
                self.logger.record(f"validation/{key}", result[key])
            self.logger.record("validation/no_route_fraction", result["no_route_fraction"])
            for key, value in result["intervention_reason_fractions"].items():
                self.logger.record(f"validation/guard_{key}", value)
            with (run/"validation.jsonl").open("a") as f:
                f.write(json.dumps(result)+"\n")
            # A stationary policy is collision-free but useless. Select curriculum
            # checkpoints by success first, then observed safety and guard reliance;
            # release qualification still requires zero observed contacts.
            score = guarded_validation_score(result)
            if training.get("dual_validation", False):
                if self.best_guarded is None or score > self.best_guarded:
                    self.best_guarded = score
                    self.model.save(run/"best_guarded")
                raw = evaluate_model(self.model, robot,
                                     replace(config, split="validation", shield=False),
                                     training.get("eval_episodes", 5), seed=10000)
                raw["timesteps"] = self.num_timesteps
                with (run/"validation_unshielded.jsonl").open("a") as f:
                    f.write(json.dumps(raw)+"\n")
                for key in ("success_rate", "collision_rate", "timeout_rate", "unsafe_command_fraction"):
                    self.logger.record(f"validation_unshielded/{key}", raw[key])
                # Optimize the weaker mode, retaining a separate guarded best
                # so an unshielded gain cannot silently erase its baseline.
                score = (min(result["success_rate"], raw["success_rate"]),
                         -result["collision_rate"]-result["cliff_rate"]-raw["collision_rate"]-raw["cliff_rate"],
                         result["success_rate"]+raw["success_rate"],
                         -raw["unsafe_command_fraction"])
                print("Unshielded validation:", {k:v for k,v in raw.items() if k != "records"}, flush=True)
            if self.best is None or score > self.best:
                self.best = score
                self.model.save(run/"best")
            print("Validation:", result, flush=True)
        def _on_step(self):
            if self.num_timesteps-self.last < training.get("eval_freq", 25000):
                return True
            self.last = self.num_timesteps
            self._evaluate()
            return True
        def _on_training_end(self):
            # Step callbacks run before PPO updates. Even an evaluation at the
            # final rollout boundary has not scored the final policy yet.
            self._evaluate()
            self.logger.dump(step=self.num_timesteps)
    checkpoint = CheckpointCallback(save_freq=max(1, training.get("checkpoint_freq", 25000)//n_envs),
                                    save_path=str(run/"checkpoints"), name_prefix="ppo")
    try:
        model.learn(total_timesteps=training.get("total_timesteps", 100000),
                    callback=[checkpoint, Validation()], reset_num_timesteps=not bool(args.resume))
        model.save(run/"final")
    except KeyboardInterrupt:
        model.save(run/"interrupted")
        print("Saved interrupted.zip; resume into a new run directory.")
        raise SystemExit(130)


if __name__ == "__main__":
    main()
