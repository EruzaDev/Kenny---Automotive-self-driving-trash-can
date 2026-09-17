"""Held-out evaluation with separate collision, cliff, timeout and intervention metrics."""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
from .config import load_config
from .env import KennyEnv


def evaluate_model(model, robot, config, episodes, seed=20000, progress_every=0):
    if episodes < 1:
        raise ValueError("episodes must be positive")
    if progress_every < 0:
        raise ValueError("progress_every must be nonnegative")
    env = KennyEnv(robot, config)
    records = []
    try:
        for i in range(episodes):
            obs, _ = env.reset(seed=seed+i)
            reward_sum = 0.
            done = False
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                reward_sum += reward
                done = terminated or truncated
            records.append({**info, "seed": seed+i, "reward": reward_sum})
            completed = i + 1
            if progress_every and (completed % progress_every == 0 or completed == episodes):
                events = {name: sum(r["event"] == name for r in records)
                          for name in ("success", "collision", "cliff", "timeout")}
                print(f"Evaluation {completed}/{episodes} | "
                      f"success={events['success']} collision={events['collision']} "
                      f"cliff={events['cliff']} timeout={events['timeout']}", flush=True)
    finally:
        env.close()
    total_steps = sum(r["steps"] for r in records)
    reasons = sorted({key for r in records for key in r["intervention_reasons"]})
    return {"episodes": episodes, "split": config.split, "stage": config.stage,
            "map_mode": config.map_mode,
            "route_clearance_weight": config.route_clearance_weight,
            "mean_steps": float(np.mean([r["steps"] for r in records])),
            "shield": config.shield, "success_rate": np.mean([r["is_success"] for r in records]).item(),
            "collision_rate": np.mean([r["event"] == "collision" for r in records]).item(),
            "cliff_rate": np.mean([r["event"] == "cliff" for r in records]).item(),
            "timeout_rate": np.mean([r["event"] == "timeout" for r in records]).item(),
            "intervention_fraction": sum(r["interventions"] for r in records)/sum(r["steps"] for r in records),
            "intervention_reason_fractions": {
                key: sum(r["intervention_reasons"].get(key, 0) for r in records)/total_steps
                for key in reasons},
            "no_route_fraction": sum(r["no_route_steps"] for r in records)/total_steps,
            "mean_reward": float(np.mean([r["reward"] for r in records])), "records": records}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", required=True)
    p.add_argument("--episodes", type=int, default=30)
    p.add_argument("--seed", type=int, default=20000)
    p.add_argument("--split", choices=["validation", "test", "stress"], default="test")
    p.add_argument("--stage", choices=["empty", "static", "mixed", "dynamic", "cliffs", "full"])
    p.add_argument("--unshielded", action="store_true", help="Simulation-only policy ablation")
    p.add_argument("--map-mode", choices=["known", "progressive"])
    p.add_argument("--route-clearance-weight", type=float,
                   help="Override planner clearance cost for a controlled evaluation")
    p.add_argument("--progress-every", type=int, default=10,
                   help="Print progress every N episodes; use 0 to disable")
    p.add_argument("--output", required=True)
    args = p.parse_args()
    from stable_baselines3 import PPO
    import torch
    torch.set_num_threads(1)
    source = Path(args.model).resolve().parent
    if source.name == "checkpoints":
        source = source.parent
    robot, config, _ = load_config(source/"config.json")
    config = replace(config, split=args.split, shield=not args.unshielded,
                     stage=args.stage or config.stage, map_mode=args.map_mode or config.map_mode)
    if args.route_clearance_weight is not None:
        config = replace(config, route_clearance_weight=args.route_clearance_weight)
    expected = json.loads((source/"contract.json").read_text())
    if expected != json.loads(json.dumps(KennyEnv(robot, config).contract())):
        p.error("Model and environment observation contracts differ")
    model = PPO.load(args.model, device="cpu")
    if args.progress_every < 0:
        p.error("--progress-every must be nonnegative")
    result = evaluate_model(model, robot, config, args.episodes, args.seed, args.progress_every)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "records"}, indent=2))


if __name__ == "__main__":
    main()
