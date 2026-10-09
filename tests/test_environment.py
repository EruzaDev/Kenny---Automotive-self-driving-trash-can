import json
from pathlib import Path
import numpy as np
import pytest
from kenny_rl.env import KennyEnv
from kenny_rl.config import EnvConfig
from kenny_rl.environment import load_environment


def example():
    return {'version':1,'name':'test','frame_id':'map','units':'meters','dictionary':'DICT_4X4_50',
            'grid':{'width':60,'height':40,'resolution':.1,'origin':[-2,-1,0],'data':[0]*2400},
            'markers':[],'obstacles':[{'kind':'wall','x':.5,'y':-.5,'width':.2,'depth':2,'bottom':0,'height':2}],
            'start':[-1,0],'goal':[3,2]}


def test_large_marker_dictionary_bounds_and_duplicates():
    from kenny_rl.environment import validate_environment
    source=example()
    source['dictionary']='DICT_4X4_1000'
    marker={'id':999,'x':-1.,'y':0.,'z':0.,'yaw':0.,'size':.2,'mount':'floor'}
    source['markers']=[marker]
    validate_environment(source)
    source['markers']=[marker,dict(marker)]
    with pytest.raises(ValueError,match='unique'):validate_environment(source)
    source['markers']=[dict(marker,id=1000)]
    with pytest.raises(ValueError,match='999'):validate_environment(source)
    source['markers']=[marker]
    source['dictionary']='DICT_4X4_50'
    with pytest.raises(ValueError,match='49'):validate_environment(source)


def test_reset_loads_custom_map_and_steps(tmp_path):
    p=tmp_path/'environment.json';p.write_text(json.dumps(example()))
    env=KennyEnv(environment=p,config=EnvConfig(stage='full'))
    observation,info=env.reset(seed=1)
    assert env.observation_space.contains(observation)
    np.testing.assert_allclose(env.world.start,[1,1])
    np.testing.assert_allclose(env.world.goal,[5,3])
    assert len(env.route)>0
    assert env.world.people.shape==(0,5)
    for _ in range(201):
        observation,reward,terminated,truncated,info=env.step(np.array([-1.,0.],dtype=np.float32))
        if terminated or truncated:break
    assert env.clutter_changes==0
    # The 6 m × 4 m editor map remains native rectangular geometry rather
    # than receiving the old artificial padding wall in the unused square area.
    assert len(env.world.kinds)==5
    env.close()


def test_bad_start_and_unknown_space_rejected():
    source=example();source['grid']['data'][10*60+10]=-1
    with pytest.raises(ValueError,match='start'):KennyEnv(environment=source).reset(seed=1)


def test_generated_maps_remain_default():
    env=KennyEnv(config=EnvConfig(stage='empty'));env.reset(seed=42)
    assert not hasattr(env.world,'map_origin')
    env.close()


def test_marker_destination_is_authoritative_and_moves_with_marker():
    source=example()
    source['markers']=[{'id':0,'x':3,'y':2,'z':0,'yaw':0,'size':.2,'mount':'floor'}]
    source['destination_marker_id']=0;source['goal']=[99,99]
    from kenny_rl.config import RobotConfig
    world=load_environment(source,RobotConfig())
    np.testing.assert_allclose(world.goal,[5,3])
    source['markers'][0]['x']=2
    world=load_environment(source,RobotConfig())
    np.testing.assert_allclose(world.goal,[4,3])
    assert world.destination_marker_id==0
    source['markers']=[]
    with pytest.raises(ValueError,match='missing'):load_environment(source,RobotConfig())


def test_wall_marker_arrival_point_is_in_front_of_its_face():
    source=example()
    source['markers']=[{'id':8,'x':2,'y':2,'z':1,'yaw':np.pi,'size':.2,'mount':'wall'}]
    source['destination_marker_id']=8
    from kenny_rl.config import RobotConfig
    world=load_environment(source,RobotConfig())
    np.testing.assert_allclose(world.goal,[3.5,3])
    np.testing.assert_allclose(world.destination_position,[4,3])
