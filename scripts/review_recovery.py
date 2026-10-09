"""Replay paired failure seeds with an existing policy and report recovery effects."""
import argparse
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import PPO

from kenny_rl.config import load_config
from kenny_rl.env import KennyEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True)
    parser.add_argument('--seeds', nargs='+', type=int, default=[20018, 20037, 20001, 20017])
    parser.add_argument('--output', required=True)
    parser.add_argument('--recovery-only', action='store_true')
    parser.add_argument('--snapshot-every', type=int, default=300)
    args = parser.parse_args()
    source = Path(args.model).parent
    robot, config, training = load_config(source/'config.json')
    torch.set_num_threads(1)
    model = PPO.load(args.model, device='cpu', custom_objects={
        'lr_schedule': lambda _: training.get('learning_rate', 1e-5),
        'clip_range': lambda _: training.get('clip_range', .05)})
    results = []
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if args.snapshot_every < 1:
        parser.error('--snapshot-every must be positive')
    for recovery in ((True,) if args.recovery_only else (False, True)):
        env = KennyEnv(robot, replace(config, split='test', recovery_enabled=recovery))
        for seed in args.seeds:
            obs, _ = env.reset(seed=seed)
            snapshots = []
            while True:
                action, _ = model.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                if env.steps % args.snapshot_every == 0 or terminated or truncated:
                    snapshots.append({'step': env.steps, 'pose': env.pose.tolist(),
                        'goal_distance': float(np.linalg.norm(env.estimate[:2]-env.world.goal)),
                        'uncertainty': env.uncertainty, 'requested': env.requested.tolist(),
                        'executed': env.executed.tolist(), 'guard': env.guard_reasons,
                        'velocity': env.measured_velocity.tolist(),
                        'depth_hits': [[float(a), float(d)] for a, d, hit in
                            zip(env.depth.angles, env.depth.ranges, env.depth.hits) if hit],
                        'route_available': bool(len(env.route))})
                if terminated or truncated:
                    break
            results.append({'recovery': recovery, 'seed': seed, **info, 'snapshots': snapshots})
            output.write_text(json.dumps(results, indent=2))
            print(recovery, seed, info['event'], info['steps'], info['intervention_reasons'], flush=True)
        env.close()


if __name__ == '__main__':
    main()
