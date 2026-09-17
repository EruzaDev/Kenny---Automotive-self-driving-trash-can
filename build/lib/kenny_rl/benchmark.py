"""Measure local simulation throughput and peak resident memory, without PyTorch."""
import argparse
from dataclasses import replace
import json
import resource
import time
import numpy as np
from .config import load_config
from .env import KennyEnv


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="configs/laptop.json")
    p.add_argument("--steps", type=int, default=1000)
    p.add_argument("--stage", default="full")
    args = p.parse_args()
    robot, config, _ = load_config(args.config)
    env = KennyEnv(robot, replace(config, stage=args.stage))
    started = time.perf_counter()
    env.reset(seed=123)
    reset_seconds = time.perf_counter()-started
    rng = np.random.default_rng(321)
    started = time.perf_counter()
    episodes = 0
    for _ in range(args.steps):
        _, _, terminated, truncated, _ = env.step(rng.uniform(-1, 1, 2))
        if terminated or truncated:
            env.reset()
            episodes += 1
    elapsed = time.perf_counter()-started
    env.close()
    print(json.dumps({"steps": args.steps, "steps_per_second": args.steps/elapsed,
                      "elapsed_seconds": elapsed, "first_reset_seconds": reset_seconds,
                      "peak_rss_mib_linux": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                      "completed_episodes": episodes, "observation_size": env.observation_space.shape[0]}, indent=2))


if __name__ == "__main__":
    main()
