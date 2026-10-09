"""Headless trained-policy replay for the environment editor."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import numpy as np
from .config import load_config
from .env import KennyEnv
from .geometry import ray_boxes, wrap_angle


def camera_marker_ids(world, pose, robot):
    """Offline geometric camera overlay, not a decoded detection or policy input.

    Unlike the legacy localization surrogate, honor surveyed marker height and
    wall-face orientation. Pixel resolution and print quality are not modeled.
    """
    result=[]
    camera=np.array([*pose[:2],robot.camera_height])
    for marker,xy in zip(world.marker_metadata,world.markers):
        delta=np.array([*xy,marker['z']])-camera
        horizontal=float(np.linalg.norm(delta[:2]))
        distance=float(np.linalg.norm(delta))
        bearing=wrap_angle(np.arctan2(delta[1],delta[0])-pose[2])
        elevation=np.rad2deg(np.arctan2(delta[2],horizontal))
        if not (robot.camera_min_range < distance < min(robot.marker_range,robot.camera_range)
                and abs(bearing) < np.deg2rad(robot.camera_hfov_deg/2)
                and abs(elevation-robot.camera_pitch_deg) < robot.camera_vfov_deg/2):
            continue
        if marker['mount']=='wall':
            yaw=marker['yaw']-world.map_origin[2]
            if np.dot(-delta[:2],[np.cos(yaw),np.sin(yaw)]) <= 0:
                continue
        elif camera[2] <= marker['z']:
            continue
        measured,_=ray_boxes(camera,(delta/distance)[None,:],world.all_boxes(),distance+.02)
        if measured[0] >= distance-.01:result.append(marker['id'])
    return result


def run_replay(environment, checkpoint, steps=1200, seed=42, model=None, controller='checkpoint'):
    if controller not in ('checkpoint', 'marker-sweep'):
        raise ValueError('controller must be checkpoint or marker-sweep')
    if type(steps) is not int or not 1 <= steps <= 3000:
        raise ValueError('steps must be 1–3000')
    if type(seed) is not int or not 0 <= seed <= 2**32-1:
        raise ValueError('seed must be an unsigned 32-bit integer')
    checkpoint=Path(checkpoint).resolve()
    source=checkpoint.parent.parent if checkpoint.parent.name=='checkpoints' else checkpoint.parent
    robot,config,training=load_config(source/'config.json')
    config=replace(config,split='test',max_steps=steps,shield=True,train_unshielded_fraction=0.)
    if controller == 'marker-sweep':
        config=replace(config,recovery_enabled=True,route_recovery_enabled=True,marker_sweep_enabled=True)
    env=KennyEnv(robot,config,environment=environment)
    try:
        expected=json.loads((source/'contract.json').read_text())
        if expected!=json.loads(json.dumps(env.contract())):
            raise ValueError('Checkpoint observation/robot contract differs from the current simulator')
        if model is None:
            from stable_baselines3 import PPO
            # Python-version-specific serialized training schedules do not affect
            # inference. Reconstruct them while retaining the saved policy weights.
            model=PPO.load(checkpoint,device='cpu',custom_objects={
                'lr_schedule':lambda _: 0., 'learning_rate':0.,
                'clip_range':lambda _: float(training.get('clip_range',.2)),
                'clip_range_vf':None})
        observation,info=env.reset(seed=seed)
        origin=env.world.map_origin
        c,s=math.cos(origin[2]),math.sin(origin[2])
        def world(p):return [float(origin[0]+c*p[0]-s*p[1]),float(origin[1]+s*p[0]+c*p[1])]
        def scan_record(scan, acquisition, max_range, age_steps=0):
            # Project observations from the estimated acquisition pose, as the
            # mapper does. Never recompute sensors from hidden scene geometry.
            return {'origin':[*world(acquisition),float(acquisition[2]+origin[2])],
                    'angles':scan.angles.tolist(),'ranges':scan.ranges.tolist(),
                    'valid':scan.valid.tolist(),'hits':scan.hits.tolist(),
                    'max_range':max_range,'age_steps':age_steps}
        def frame():
            distance=float(np.linalg.norm(env.pose[:2]-env.world.destination_position))
            return {'step':env.steps,'pose':[*world(env.pose),float(env.pose[2]+origin[2])],
                    'estimate':[*world(env.estimate),float(env.estimate[2]+origin[2])],
                    'route':[world(p) for p in env.route],
                    'intervened':bool(env.intervened),'event':info['event'],
                    'distance_to_destination_m':distance,'near_goal':distance<=5.,
                    'guard_reasons':list(env.guard_reasons),
                    'camera_visible_marker_ids':camera_marker_ids(env.world,env.pose,robot),
                    'marker_localization_updated':env.steps > 0 and env.marker_age == 0,
                    'sensors':{
                        'lidar':scan_record(env.lidar,env.lidar_estimate,robot.lidar_range,
                                            env.steps-env.last_lidar_step),
                        'depth':scan_record(env.depth,env.estimate,robot.camera_range),
                        'floor':scan_record(env.floor,env.estimate,robot.camera_range),
                        'camera_hfov_deg':robot.camera_hfov_deg}}
        frames=[frame()];reward_sum=0.
        for _ in range(steps):
            action,_=model.predict(observation,deterministic=True)
            observation,reward,terminated,truncated,info=env.step(action)
            reward_sum+=reward;frames.append(frame())
            if terminated or truncated:break
        return {'frames':frames,'info':info,'dt':config.dt,'seed':seed,'model':str(checkpoint),
                'reward':reward_sum,'robot_radius':robot.radius,'robot_height':robot.height,
                'map_mode':config.map_mode,'shield':True,'controller':controller,
                'recovery_enabled':config.recovery_enabled,
                'route_recovery_enabled':config.route_recovery_enabled,
                'marker_sweep_enabled':config.marker_sweep_enabled,
                'start':world(env.world.start),'goal':world(env.world.goal),
                'destination':world(env.world.destination_position),
                'destination_marker_id':env.world.destination_marker_id,'near_goal_threshold_m':5.}
    finally:env.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--environment',required=True);p.add_argument('--model',required=True)
    p.add_argument('--steps',type=int,default=1200);p.add_argument('--seed',type=int,default=42);p.add_argument('--output',required=True)
    p.add_argument('--controller',choices=('checkpoint','marker-sweep'),default='checkpoint')
    args=p.parse_args()
    if not 1<=args.steps<=3000:p.error('steps must be 1–3000')
    import torch
    torch.set_num_threads(1)
    result=run_replay(args.environment,args.model,args.steps,args.seed,controller=args.controller)
    Path(args.output).write_text(json.dumps(result,allow_nan=False))
    print(json.dumps(result['info']),flush=True)


if __name__=='__main__':main()
