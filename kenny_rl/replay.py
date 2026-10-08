"""Headless trained-policy replay for the environment editor."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import numpy as np
from .config import load_config
from .env import KennyEnv


def run_replay(environment, checkpoint, steps=1200, seed=42, model=None):
    if type(steps) is not int or not 1 <= steps <= 3000:
        raise ValueError('steps must be 1–3000')
    if type(seed) is not int or not 0 <= seed <= 2**32-1:
        raise ValueError('seed must be an unsigned 32-bit integer')
    checkpoint=Path(checkpoint).resolve()
    source=checkpoint.parent.parent if checkpoint.parent.name=='checkpoints' else checkpoint.parent
    robot,config,training=load_config(source/'config.json')
    config=replace(config,split='test',max_steps=steps,shield=True,train_unshielded_fraction=0.)
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
        def frame():
            distance=float(np.linalg.norm(env.pose[:2]-env.world.destination_position))
            return {'step':env.steps,'pose':[*world(env.pose),float(env.pose[2]+origin[2])],
                    'estimate':[*world(env.estimate),float(env.estimate[2]+origin[2])],
                    'route':[world(p) for p in env.route],
                    'intervened':bool(env.intervened),'event':info['event'],
                    'distance_to_destination_m':distance,'near_goal':distance<=5.}
        frames=[frame()];reward_sum=0.
        for _ in range(steps):
            action,_=model.predict(observation,deterministic=True)
            observation,reward,terminated,truncated,info=env.step(action)
            reward_sum+=reward;frames.append(frame())
            if terminated or truncated:break
        return {'frames':frames,'info':info,'dt':config.dt,'seed':seed,'model':str(checkpoint),
                'reward':reward_sum,'robot_radius':robot.radius,'robot_height':robot.height,
                'map_mode':config.map_mode,'shield':True,
                'start':world(env.world.start),'goal':world(env.world.goal),
                'destination':world(env.world.destination_position),
                'destination_marker_id':env.world.destination_marker_id,'near_goal_threshold_m':5.}
    finally:env.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--environment',required=True);p.add_argument('--model',required=True)
    p.add_argument('--steps',type=int,default=1200);p.add_argument('--seed',type=int,default=42);p.add_argument('--output',required=True)
    args=p.parse_args()
    if not 1<=args.steps<=3000:p.error('steps must be 1–3000')
    import torch
    torch.set_num_threads(1)
    result=run_replay(args.environment,args.model,args.steps,args.seed)
    Path(args.output).write_text(json.dumps(result,allow_nan=False))
    print(json.dumps(result['info']),flush=True)


if __name__=='__main__':main()
