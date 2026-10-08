import json
import numpy as np
import pytest
from kenny_rl.config import RobotConfig, EnvConfig, serialize
from kenny_rl.env import KennyEnv
from kenny_rl.replay import run_replay


def setup_run(tmp_path):
    robot=RobotConfig();config=EnvConfig(stage='empty',domain_randomization=False,dropout=0,marker_dropout=0,sensor_noise=0)
    (tmp_path/'config.json').write_text(json.dumps(serialize(robot,config)))
    (tmp_path/'contract.json').write_text(json.dumps(KennyEnv(robot,config).contract()))
    environment={'version':1,'name':'replay test','frame_id':'map','units':'meters','dictionary':'DICT_4X4_50',
        'grid':{'width':40,'height':40,'resolution':.1,'origin':[-2,-3,0],'data':[0]*1600},
        'start':[-1,-2],'goal':[1,0],'markers':[],'obstacles':[]}
    return environment,tmp_path/'best.zip'


class StopPolicy:
    def __init__(self):self.calls=0
    def predict(self,observation,deterministic):
        assert deterministic
        assert np.isfinite(observation).all()
        self.calls+=1
        return np.array([-1.,0.],dtype=np.float32),None


def test_real_environment_replay_records_world_coordinates_and_timeout(tmp_path):
    environment,checkpoint=setup_run(tmp_path);policy=StopPolicy()
    result=run_replay(environment,checkpoint,steps=5,seed=7,model=policy)
    assert policy.calls==5
    assert len(result['frames'])==6
    assert result['info']['event']=='timeout'
    assert result['info']['steps']==5
    assert result['shield']
    np.testing.assert_allclose(result['frames'][0]['pose'][:2],environment['start'])
    np.testing.assert_allclose(result['frames'][-1]['pose'][:2],environment['start'])
    assert result['frames'][0]['route']
    assert result['frames'][-1]['event']=='timeout'
    json.dumps(result,allow_nan=False)


def test_replay_rejects_incompatible_contract_and_bad_start(tmp_path):
    environment,checkpoint=setup_run(tmp_path)
    (tmp_path/'contract.json').write_text('{}')
    with pytest.raises(ValueError,match='contract'):run_replay(environment,checkpoint,model=StopPolicy())
    environment,checkpoint=setup_run(tmp_path);environment['start']=[99,99]
    with pytest.raises(ValueError,match='start'):run_replay(environment,checkpoint,model=StopPolicy())


def test_replay_validates_limits(tmp_path):
    environment,checkpoint=setup_run(tmp_path)
    for steps in (0,3001,1.5):
        with pytest.raises(ValueError,match='steps'):run_replay(environment,checkpoint,steps=steps,model=StopPolicy())


def test_near_uses_true_horizontal_distance_to_marker_not_wall_approach(tmp_path):
    environment,checkpoint=setup_run(tmp_path)
    # Robot starts at [-1,-2]; marker is 5.25m away, approach point is 4.75m away.
    environment['grid'].update(width=120,height=120,data=[0]*14400)
    environment['markers']=[{'id':0,'x':4.25,'y':-2,'z':1,'yaw':np.pi,'size':.2,'mount':'wall'}]
    environment['destination_marker_id']=0
    result=run_replay(environment,checkpoint,steps=1,seed=42,model=StopPolicy())
    assert result['destination_marker_id']==0
    np.testing.assert_allclose(result['destination'],[4.25,-2])
    np.testing.assert_allclose(result['goal'],[3.75,-2])
    assert result['frames'][0]['distance_to_destination_m']==pytest.approx(5.25)
    assert not result['frames'][0]['near_goal']
    # Exactly 5m is near; this remains based on truth even with a separate pose estimate.
    environment['markers'][0]['x']=4
    result=run_replay(environment,checkpoint,steps=1,seed=42,model=StopPolicy())
    assert result['frames'][0]['distance_to_destination_m']==pytest.approx(5.)
    assert result['frames'][0]['near_goal']
