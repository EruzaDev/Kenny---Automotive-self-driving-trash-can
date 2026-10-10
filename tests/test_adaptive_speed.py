from dataclasses import replace

import numpy as np
import pytest

from kenny_rl.config import EnvConfig, RobotConfig
from kenny_rl.env import KennyEnv


@pytest.fixture
def clear_env():
    env = KennyEnv(RobotConfig(max_speed=.6), EnvConfig(stage="mixed",
                  adaptive_speed_enabled=True, command_delay_max_steps=0,
                  domain_randomization=False, dropout=0., marker_dropout=0.))
    env.reset(seed=42)
    env.pose[2] = env.estimate[2] = 0.
    origin = env.estimate[:2].copy()
    env.route = origin + np.array([[0., 0.], [10., 0.]])
    env.world.goal = origin + [10., 0.]
    for scan, distance in ((env.lidar, env.robot.lidar_range),
                           (env.depth, env.robot.camera_range),
                           (env.floor, env.robot.camera_range)):
        scan.valid[:] = True
        scan.hits[:] = False
        scan.ranges[:] = distance
    env.down_valid[:] = True
    env.down_hazard[:] = False
    yield env
    env.close()


def obstacle(env, distance, lateral=0.):
    angle = np.arctan2(lateral, distance)
    index = int(np.argmin(np.abs(env.lidar.angles-angle)))
    env.lidar.hits[index] = True
    env.lidar.ranges[index] = np.hypot(distance, lateral)


def test_clear_straight_route_cruises_but_explicit_stop_is_preserved(clear_env):
    assert clear_env._adaptive_speed_target(np.array([.1, 0.]))[0] == pytest.approx(.6)
    assert clear_env._adaptive_speed_target(np.array([0., 0.]))[0] == 0.
    # Recovery maneuvers cannot be boosted into cruise commands.
    assert clear_env._adaptive_speed_target(np.array([.12, 0.]), allow_cruise=False)[0] == .12


def test_obstacle_clearance_slows_and_then_brakes(clear_env):
    obstacle(clear_env, .6)
    slow = clear_env._adaptive_speed_target(np.array([.6, 0.]))[0]
    assert 0. < slow < .6
    obstacle(clear_env, .15)
    assert clear_env._adaptive_speed_target(np.array([.6, 0.]))[0] == 0.


def test_approaching_people_and_crossing_corridor_reduce_speed(clear_env):
    obstacle(clear_env, 2.)
    stationary = clear_env._adaptive_speed_target(np.array([.6, 0.]))[0]
    clear_env.config = replace(clear_env.config, stage="dynamic")
    assert clear_env._adaptive_speed_target(np.array([.6, 0.]))[0] < stationary
    clear_env.lidar.hits[:] = False
    obstacle(clear_env, .5, lateral=.8)
    assert 0. < clear_env._adaptive_speed_target(np.array([.6, 0.]))[0] < .6


def test_grid_offset_allows_steering_without_cruise_boost(clear_env):
    origin = clear_env.estimate[:2]
    clear_env.route = origin + [[0., .125], [10., .125]]
    speed = clear_env._adaptive_speed_target(np.array([.2, .25]))[0]
    assert speed == pytest.approx(.2)


def test_side_return_does_not_become_zero_forward_clearance(clear_env):
    clear_env.config = replace(clear_env.config, stage="dynamic")
    obstacle(clear_env, .05, lateral=.8)
    speed = clear_env._adaptive_speed_target(np.array([.6, 0.]))[0]
    assert 0. < speed < .6
    obstacle(clear_env, .15)
    assert clear_env._adaptive_speed_target(np.array([.6, 0.]))[0] == 0.


def test_curve_and_unknown_sensor_coverage_prevent_cruise(clear_env):
    origin = clear_env.estimate[:2]
    clear_env.route = origin + [[0., 0.], [.5, 0.], [.5, 3.]]
    assert clear_env._adaptive_speed_target(np.array([.6, 0.]))[0] < .6
    clear_env.lidar.valid[np.argmin(np.abs(clear_env.lidar.angles))] = False
    assert clear_env._adaptive_speed_target(np.array([.6, 0.]))[0] == 0.


def test_speed_limit_reserves_latency_braking_and_approach(clear_env):
    limit = clear_env._clearance_speed(2., closing_speed=.8)
    a = clear_env.robot.acceleration*clear_env.config.acceleration_scale_range[0]
    latency = clear_env.config.dt+1/clear_env.robot.lidar_hz
    stopping = limit*latency+limit**2/(2*a)+.8*(latency+limit/a)
    assert stopping == pytest.approx(2.)
    clear_env.config = replace(clear_env.config, command_delay_max_steps=5)
    assert clear_env._clearance_speed(2., closing_speed=.8) < limit


def test_cruise_accelerates_gradually_and_emergency_stop_keeps_braking_bounds(clear_env):
    # Isolate the dynamics from new sensor acquisitions while exercising step.
    from unittest.mock import patch
    with patch.object(clear_env, "_sense"), patch.object(clear_env, "_replan"):
        clear_env.step(np.array([0., 0.], dtype=np.float32))
        assert 0. < clear_env.velocity[0] <= clear_env.robot.acceleration*clear_env.config.dt
        obstacle(clear_env, .15)
        previous = clear_env.velocity[0]
        clear_env.step(np.array([1., 0.], dtype=np.float32))
        assert 0. <= clear_env.velocity[0] < previous
        assert previous-clear_env.velocity[0] <= clear_env.robot.acceleration*clear_env.config.dt+1e-9


def test_adaptive_controller_is_explicit_in_contract(clear_env):
    contract = clear_env.contract()
    assert contract["adaptive_speed_controller"] == "route-sensor-cruise-v2"
    clear_env.config = replace(clear_env.config, adaptive_speed_enabled=False)
    assert "adaptive_speed_controller" not in clear_env.contract()


def test_demonstrations_include_slowdown_without_double_counting(clear_env):
    from kenny_rl.demo import route_action
    obstacle(clear_env, .6)
    action = route_action(clear_env)
    assert 0. < (action[0]+1)*clear_env.robot.max_speed/2 < .6
    assert clear_env.speed_governor_events == 0


def test_bootstrap_stage_is_validated():
    from kenny_rl.config import validate_training
    validate_training({"behavior_cloning_stage": "empty"})
    with pytest.raises(ValueError, match="behavior_cloning_stage"):
        validate_training({"behavior_cloning_stage": "bad"})


def test_speed_transfer_rejects_sensor_or_dynamics_changes(clear_env):
    from copy import deepcopy
    from kenny_rl.train import validate_speed_transfer
    target = clear_env.contract()
    old = deepcopy(target)
    old.pop("adaptive_speed_controller")
    old["robot"]["max_speed"] = .3
    validate_speed_transfer(old, target)
    for section, key in [("robot", "lidar_range"), ("robot", "acceleration")]:
        changed = deepcopy(old)
        changed[section][key] += .1
        with pytest.raises(ValueError, match="matching sensors"):
            validate_speed_transfer(changed, target)


def test_validation_does_not_prefer_idle_for_fewer_interventions():
    from kenny_rl.train import guarded_validation_score
    idle = dict(success_rate=0., collision_rate=0., cliff_rate=0.,
                intervention_fraction=0., mean_reward=-18.)
    progressing = dict(idle, intervention_fraction=.4, mean_reward=-4.)
    assert guarded_validation_score(progressing) > guarded_validation_score(idle)
    assert guarded_validation_score(dict(progressing, collision_rate=.1)) < guarded_validation_score(idle)
    assert guarded_validation_score(dict(idle, success_rate=.5)) > guarded_validation_score(progressing)


def test_checkpoint_review_selects_from_validation_not_test_data(tmp_path):
    import importlib.util
    import json
    from pathlib import Path
    spec = importlib.util.spec_from_file_location("speed_experiment", Path(__file__).parents[1]/"scripts/train_faster_noise.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    initial = dict(timesteps=0, success_rate=0., collision_rate=0., cliff_rate=0.,
                   intervention_fraction=.4, mean_reward=-4.)
    final = dict(initial, timesteps=8192, intervention_fraction=0., mean_reward=-18.)
    (tmp_path/"validation.jsonl").write_text(json.dumps(initial)+"\n"+json.dumps(final)+"\n")
    (tmp_path/"initialized.zip").write_bytes(b"moving policy")
    (tmp_path/"final.zip").write_bytes(b"idle policy")
    module.review_initialized_checkpoint(tmp_path)
    assert (tmp_path/"best_guarded.zip").read_bytes() == b"moving policy"
    assert json.loads((tmp_path/"checkpoint-selection.json").read_text())["held_out_data_used"] is False


@pytest.mark.parametrize("value", [1, "yes", None])
def test_adaptive_flag_requires_boolean(value):
    with pytest.raises(ValueError, match="adaptive_speed_enabled"):
        EnvConfig(adaptive_speed_enabled=value)
