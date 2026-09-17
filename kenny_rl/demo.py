"""Preview generated worlds without training; optional checkpoint playback."""
import argparse
from dataclasses import replace
from pathlib import Path
import numpy as np
from .config import load_config
from .env import KennyEnv


def route_action(env):
    """Debug route follower, not a navigation performance baseline."""
    if not len(env.route):
        return np.array([-1., 0.], dtype=np.float32)
    point = env._lookahead()[0][0]
    heading = np.arctan2(point[1], point[0])
    goal_distance = np.linalg.norm(env.world.goal-env.estimate[:2])
    speed = env.approach_speed(goal_distance) * max(0., np.cos(heading))
    if goal_distance < .23:
        return np.array([-1., 0.], dtype=np.float32)
    return np.array([2*speed/env.robot.max_speed-1, np.clip(2*heading/env.robot.max_turn_rate, -1, 1)], dtype=np.float32)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="configs/laptop.json")
    p.add_argument("--stage", default="full")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--steps", type=int, default=500)
    p.add_argument("--snapshot", help="Save one PNG without opening a window")
    p.add_argument("--model")
    p.add_argument("--map-mode", choices=["known", "progressive"])
    p.add_argument("--start", nargs=2, type=float, metavar=("X", "Y"))
    p.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"))
    args = p.parse_args()
    robot, config, _ = load_config(args.config)
    if args.model:
        source = Path(args.model).resolve().parent
        if source.name == "checkpoints":
            source = source.parent
        robot, config, _ = load_config(source/"config.json")
    env = KennyEnv(robot, replace(config, stage=args.stage, split="test",
                                 map_mode=args.map_mode or config.map_mode),
                   render_mode="rgb_array" if args.snapshot else "human")
    options = {k: v for k, v in {"start": args.start, "goal": args.goal}.items() if v is not None}
    obs, _ = env.reset(seed=args.seed, options=options)
    try:
        if args.snapshot:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            Path(args.snapshot).parent.mkdir(parents=True, exist_ok=True)
            plt.imsave(args.snapshot, env.render())
            return
        model = None
        if args.model:
            from stable_baselines3 import PPO
            model = PPO.load(args.model, device="cpu")
        for _ in range(args.steps):
            action = model.predict(obs, deterministic=True)[0] if model else route_action(env)
            obs, _, terminated, truncated, info = env.step(action)
            env.render()
            if terminated or truncated:
                print(info)
                break
    finally:
        env.close()


if __name__ == "__main__":
    main()
