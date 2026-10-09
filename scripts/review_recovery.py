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
    parser.add_argument('--route-recovery', action='store_true')
    parser.add_argument('--marker-sweep', action='store_true')
    parser.add_argument('--world-width', type=float, help='Nominal generated room width in metres')
    parser.add_argument('--world-height', type=float, help='Nominal generated room height in metres')
    parser.add_argument('--snapshot-every', type=int, default=300)
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error('Output already exists; choose a new filename to preserve previous replays')
    if args.snapshot_every < 1:
        parser.error('--snapshot-every must be positive')
    source = Path(args.model).parent
    robot, config, training = load_config(source/'config.json')
    try:
        config = replace(config,
                         world_width=args.world_width if args.world_width is not None else config.world_width,
                         world_height=args.world_height if args.world_height is not None else config.world_height)
    except ValueError as error:
        parser.error(str(error))
    torch.set_num_threads(1)
    model = PPO.load(args.model, device='cpu', custom_objects={
        'lr_schedule': lambda _: training.get('learning_rate', 1e-5),
        'clip_range': lambda _: training.get('clip_range', .05)})
    results = []
    output.parent.mkdir(parents=True, exist_ok=True)
    for recovery in ((True,) if args.recovery_only else (False, True)):
        env = KennyEnv(robot, replace(config, split='test', recovery_enabled=recovery,
                                     route_recovery_enabled=(args.route_recovery or args.marker_sweep) and recovery,
                                     marker_sweep_enabled=args.marker_sweep and recovery))
        for seed in args.seeds:
            obs, _ = env.reset(seed=seed)
            snapshots = []
            while True:
                action, _ = model.predict(obs, deterministic=True)
                obs, _, terminated, truncated, info = env.step(action)
                if env.steps % args.snapshot_every == 0 or terminated or truncated:
                    snapshots.append({'step': env.steps, 'pose': env.pose.tolist(),
                        'estimated_pose': env.estimate.tolist(),
                        'goal_distance': float(np.linalg.norm(env.estimate[:2]-env.world.goal)),
                        'route_remaining': env._remaining(),
                        'route_lookahead': env._lookahead()[0].tolist(),
                        'uncertainty': env.uncertainty, 'requested': env.requested.tolist(),
                        'sensor_valid_fractions': {
                            'depth': float(env.depth.valid.mean()),
                            'floor': float(env.floor.valid.mean()),
                            'lidar': float(env.lidar.valid.mean()),
                            'downward': float(env.down_valid.mean())},
                        'marker_age': env.marker_age,
                        'recovery_active': env.route_recovery_active,
                        'recovery_ticks': env.route_recovery_ticks,
                        'recovery_retry_step': env.route_recovery_retry_step,
                        'recovery_scan_armed': env.route_recovery_scan_armed,
                        'recovery_localization_only': env.route_recovery_localization_only,
                        'recovery_sweep_angle': env.route_recovery_sweep_angle,
                        'depth_memory_count': len(env.depth_memory),
                        'executed': env.executed.tolist(), 'guard': env.guard_reasons,
                        'velocity': env.measured_velocity.tolist(),
                        'depth_hits': [[float(a), float(d)] for a, d, hit in
                            zip(env.depth.angles, env.depth.ranges, env.depth.hits) if hit],
                        'route_available': bool(len(env.route))})
                if terminated or truncated:
                    break
            results.append({'recovery': recovery, 'route_recovery': (args.route_recovery or args.marker_sweep) and recovery,
                            'marker_sweep': args.marker_sweep and recovery,
                            'world_width': config.world_width or config.world_size,
                            'world_height': config.world_height or config.world_size,
                            'actual_world_width': env.world.width,
                            'actual_world_height': env.world.height,
                            'seed': seed, **info, 'snapshots': snapshots})
            output.write_text(json.dumps(results, indent=2))
            print(recovery, seed, info['event'], info['steps'], info['intervention_reasons'], flush=True)
        env.close()


if __name__ == '__main__':
    main()
