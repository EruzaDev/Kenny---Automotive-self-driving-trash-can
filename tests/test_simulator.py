from dataclasses import replace
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env
from kenny_rl import KennyEnv
from kenny_rl.config import RobotConfig, EnvConfig
from kenny_rl.geometry import ray_boxes, circle_boxes, circle_rects
from kenny_rl.planner import GridPlanner
from kenny_rl.world import World
from kenny_rl import sensors


def scene(boxes=(), cliffs=()):
    return World(12., np.asarray(boxes, float).reshape(-1, 6), ["wall"]*len(boxes),
                 np.asarray(cliffs, float).reshape(-1, 4), np.empty((0, 2)),
                 np.empty((0, 5)), np.array([1., 1.]), np.array([8., 9.]))


def test_gym_contract_and_reproducibility():
    env = KennyEnv(config=EnvConfig(stage="empty", max_steps=20))
    check_env(env, skip_render_check=True)
    first, _ = env.reset(seed=12)
    transition = env.step(np.array([.3, -.2]))
    second, _ = env.reset(seed=12)
    repeated = env.step(np.array([.3, -.2]))
    np.testing.assert_array_equal(first, second)
    np.testing.assert_array_equal(transition[0], repeated[0])
    assert transition[1:] == repeated[1:]


@pytest.mark.parametrize("stage", ["empty", "static", "mixed", "dynamic", "cliffs", "full"])
def test_all_stages_emit_finite_observations(stage):
    env = KennyEnv(config=EnvConfig(stage=stage, max_steps=12))
    for seed in range(3):
        obs, _ = env.reset(seed=seed)
        assert env.observation_space.contains(obs)
        assert not circle_boxes(env.pose[:2], env.robot.radius, env.world.all_boxes(), env.robot.height)
        assert not circle_rects(env.pose[:2], env.robot.radius, env.world.cliffs)
        for _ in range(12):
            obs, reward, terminated, truncated, info = env.step(np.array([.5, .3]))
            assert env.observation_space.contains(obs)
            assert np.isfinite(reward)
            if terminated or truncated:
                break
        assert terminated or truncated
        with pytest.raises(RuntimeError):
            env.step(np.zeros(2))


def test_height_blind_lidar_and_depth_obstacles():
    robot = RobotConfig()
    pose = np.array([1., 1., 0.])
    # Hanging obstacle intersects the bin but is above the horizontal scan.
    world = scene([[2., .6, .31, 2.3, 1.4, .44]])
    scan = sensors.lidar(world, pose, robot, np.random.default_rng(0), 0, 0)
    depth = sensors.depth(world, pose, robot, np.random.default_rng(0), 0, 0)
    assert not scan.hits.any()
    assert depth.hits.any()
    assert circle_boxes([2.1, 1.], robot.radius, world.boxes, robot.height)
    # A low bag is also below the scan.
    world = scene([[1.7, .6, 0., 2.1, 1.4, .15]])
    assert not sensors.lidar(world, pose, robot, np.random.default_rng(0), 0, 0).hits.any()
    assert sensors.depth(world, pose, robot, np.random.default_rng(0), 0, 0).hits.any()


def test_overhead_object_above_robot_does_not_collide():
    robot = RobotConfig()
    assert not circle_boxes([1., 1.], robot.radius, [[.5, .5, .8, 1.5, 1.5, 1.]], robot.height)


def test_rays_parallel_axes_and_nearest_occlusion():
    box = np.array([[2., 0., 0., 3., 1., 1.], [4., 0., 0., 5., 1., 1.]])
    ranges, indices = ray_boxes([0., 0., .5], np.array([[1., 0., 0.], [0., 1., 0.]]), box, 10.)
    np.testing.assert_allclose(ranges, [2., 10.])
    np.testing.assert_array_equal(indices, [0, -1])


def test_cliffs_and_invalid_floor_are_distinct():
    robot = RobotConfig()
    world = scene(cliffs=[[1.4, .5, 3., 1.5]])
    pose = np.array([1., 1., 0.])
    floor = sensors.floor_scan(world, pose, robot, np.random.default_rng(1), 0.)
    assert floor.hits.any()
    invalid = sensors.floor_scan(world, pose, robot, np.random.default_rng(1), 1.)
    assert not invalid.valid.any()
    assert not invalid.hits.any()
    np.testing.assert_array_equal(invalid.ranges, robot.camera_range)


def test_invalid_ranges_do_not_leak_hidden_obstacles():
    robot = RobotConfig()
    pose = np.array([1., 1., 0.])
    world = scene([[2., .6, 0., 2.3, 1.4, 1.]])
    for function, maximum in ((sensors.lidar, robot.lidar_range), (sensors.depth, robot.camera_range)):
        scan = function(world, pose, robot, np.random.default_rng(1), 0., 1.)
        assert not scan.valid.any() and not scan.hits.any()
        np.testing.assert_array_equal(scan.ranges, maximum)


def test_marker_visibility_requires_fov_and_line_of_sight():
    robot = RobotConfig()
    world = scene()
    world.markers = np.array([[2., 1.]])
    pose = np.array([1., 1., 0.])
    assert sensors.marker_visible(world, pose, robot, np.random.default_rng(0), 0.)
    assert not sensors.marker_visible(world, [1., 1., np.pi], robot, np.random.default_rng(0), 0.)
    world.boxes = np.array([[1.4, .5, 0., 1.6, 1.5, 1.]])
    assert not sensors.marker_visible(world, pose, robot, np.random.default_rng(0), 0.)


def test_planner_detours_and_temporary_memory_expires():
    planner = GridPlanner(6., .25, .16)
    planner.set_walls([[2.5, 0., 0., 2.75, 4., 1.]])
    route = planner.path([1., 1.], [4., 1.])
    assert len(route) and route[:, 1].max() > 4.
    planner.observe([[4., 1.]], step=0, ttl=10)
    assert not len(planner.path([1., 1.], [4., 1.], step=1))
    assert len(planner.path([1., 1.], [4., 1.], step=10))


def test_splits_are_different_and_reset_clears_history():
    a = KennyEnv(config=EnvConfig(split="train"))
    b = KennyEnv(config=EnvConfig(split="test"))
    obs, _ = a.reset(seed=22)
    b.reset(seed=22)
    assert not np.array_equal(a.world.start, b.world.start)
    a.step(np.ones(2))
    reset, _ = a.reset(seed=22)
    np.testing.assert_array_equal(reset, obs)
    assert a.steps == a.interventions == 0


def test_invalid_action_rejected_and_time_limit_is_truncation():
    env = KennyEnv(config=EnvConfig(stage="empty", max_steps=1))
    env.reset(seed=0)
    with pytest.raises(ValueError):
        env.step([np.nan, 0.])
    _, _, terminated, truncated, info = env.step([-1., 0.])
    assert not terminated and truncated and info["event"] == "timeout"


def test_downward_interlock_does_not_need_forward_depth():
    env = KennyEnv(config=EnvConfig(stage="empty", domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=5)
    env.down_hazard[:] = [False, True, False]
    target, stopped = env._guard(np.array([.2, 0.]))
    assert stopped
    np.testing.assert_array_equal(target, [0., 0.])


def test_nominal_wheel_speed_and_user_dimensions():
    r = RobotConfig()
    assert r.width == r.length == .23 and r.height == .485
    assert r.track_width == .235
    assert r.wheel_surface_speed == pytest.approx(1.394867138193868)


def test_clutter_changes_do_not_spawn_on_robot_or_goal():
    env = KennyEnv(config=EnvConfig(stage="full"))
    env.reset(seed=19)
    changed = env.world.change_clutter(np.random.default_rng(22), env.pose, env.robot)
    assert changed
    assert "bag" in env.world.kinds
    assert not circle_boxes(env.pose[:2], env.robot.radius, env.world.boxes, env.robot.height)
    assert not circle_boxes(env.world.goal, env.robot.radius, env.world.boxes, env.robot.height)


def test_stress_layout_has_reachable_initial_mission():
    env = KennyEnv(config=EnvConfig(stage="full", split="stress"))
    obs, _ = env.reset(seed=40)
    assert env.world.size >= 12.*1.1
    assert env.observation_space.contains(obs)


def test_guard_uses_observations_not_hidden_geometry():
    env = KennyEnv(config=EnvConfig(stage="empty", dropout=0., sensor_noise=0.))
    env.reset(seed=12)
    env.lidar.hits[:] = False
    env.lidar.valid[:] = True
    env.depth.hits[:] = False
    env.depth.valid[:] = True
    env.floor.hits[:] = False
    env.floor.valid[:] = True
    env.down_hazard[:] = False
    env.down_valid[:] = True
    before = env._guard(np.array([.2, 0.]))
    # Changing simulator geometry without publishing a sensor sample must not
    # grant the guard supernatural collision knowledge.
    x, y = env.pose[:2]
    env.world.boxes = np.array([[x-.1, y-.1, 0., x+.1, y+.1, 1.]])
    after = env._guard(np.array([.2, 0.]))
    np.testing.assert_array_equal(before[0], after[0])
    assert before[1] == after[1]


def test_requested_goal_coordinates_and_invalid_start():
    env = KennyEnv(config=EnvConfig(stage="empty", domain_randomization=False))
    env.reset(seed=1, options={"start": [1., 1.], "goal": [8., 9.], "heading": 0.})
    np.testing.assert_array_equal(env.world.goal, [8., 9.])
    np.testing.assert_array_equal(env.pose, [1., 1., 0.])
    assert len(env.route)
    with pytest.raises(ValueError, match="collision-free"):
        env.reset(seed=1, options={"start": [0., 0.]})


def test_guard_does_not_observe_hidden_braking_parameter():
    env = KennyEnv(config=EnvConfig(stage="empty", dropout=0.))
    env.reset(seed=12)
    for scan in (env.lidar, env.depth, env.floor):
        scan.valid[:] = True
        scan.hits[:] = False
    env.down_hazard[:] = False
    env.down_valid[:] = True
    index = np.argmin(abs(env.lidar.angles))
    env.lidar.hits[index] = True
    env.lidar.ranges[index] = .42
    env.velocity[:] = [.3, 0.]
    env.accel_scale = .7
    slow = env._guard(np.array([.3, 0.]))
    env.accel_scale = 1.1
    fast = env._guard(np.array([.3, 0.]))
    assert slow[1] and fast[1]
    np.testing.assert_array_equal(slow[0], fast[0])


def test_burst_outage_blocks_motion_then_recovers_and_reset_clears_it():
    env = KennyEnv(config=EnvConfig(stage="empty", dropout=0., sensor_outage_probability=0.))
    env.reset(seed=12)
    env.outages["depth"] = 3
    for _ in range(3):
        env._sense()
        assert not env.depth.valid.any() and not env.floor.valid.any()
        assert not env.depth.hits.any() and not env.floor.hits.any()
        np.testing.assert_array_equal(env.depth.ranges, env.robot.camera_range)
        assert env._guard(np.array([.2, 0.]))[1]
    env._sense()
    assert env.depth.valid.any()
    env.outages["depth"] = 5
    env.reset(seed=12)
    assert env.outages == {"depth": 0, "lidar": 0}


def test_configurable_disturbances_are_seeded_and_serializable():
    import json
    from kenny_rl.config import serialize
    config = EnvConfig(stage="empty", motor_gain_range=(.8, .8), slip_range=(.9, .9),
                       acceleration_scale_range=(.6, .6), command_delay_max_steps=4,
                       sensor_outage_probability=.4)
    config = EnvConfig(**json.loads(json.dumps(serialize(RobotConfig(), config)))["environment"])
    env = KennyEnv(config=config)
    first, _ = env.reset(seed=123)
    np.testing.assert_allclose(env.gain, [.8, .8])
    np.testing.assert_allclose(env.slip, [.9, .9])
    assert env.accel_scale == .6 and 0 <= env.delay <= 4
    second, _ = env.reset(seed=123)
    np.testing.assert_array_equal(first, second)


@pytest.mark.parametrize("kwargs", [{"acceleration_scale_range": [0., 1.]},
                                    {"command_delay_max_steps": -1},
                                    {"sensor_outage_steps": [5, 2]},
                                    {"sensor_outage_probability": 1.}])
def test_invalid_disturbance_parameters_rejected(kwargs):
    with pytest.raises(ValueError):
        EnvConfig(**kwargs)


def test_evaluation_reports_periodic_and_final_progress(capsys):
    from kenny_rl.evaluate import evaluate_model

    class StopModel:
        def predict(self, observation, deterministic=True):
            return np.array([-1., 0.]), None

    result = evaluate_model(StopModel(), RobotConfig(),
                            EnvConfig(stage="empty", split="test", max_steps=1),
                            episodes=3, progress_every=2)
    output = capsys.readouterr().out
    assert "Evaluation 2/3" in output
    assert "Evaluation 3/3" in output
    assert result["episodes"] == 3


def test_timeout_has_an_explicit_terminal_penalty():
    env = KennyEnv(config=EnvConfig(stage="empty", max_steps=1,
                                    domain_randomization=False, dropout=0.,
                                    marker_dropout=0., sensor_noise=0.))
    env.reset(seed=7)
    _, reward, terminated, truncated, info = env.step([-1., 0.])
    assert not terminated and truncated and info["event"] == "timeout"
    assert reward < -5


def test_correct_progress_outweighs_action_change_penalty():
    env = KennyEnv(config=EnvConfig(stage="empty", max_steps=2,
                                    domain_randomization=False, dropout=0.,
                                    marker_dropout=0., sensor_noise=0.))
    env.reset(seed=7, options={"heading": 0.})
    env.route = np.array([env.pose[:2], env.pose[:2] + [.5, 0.]])
    env.previous_remaining = env._remaining()
    _, reward, _, _, _ = env.step([1., 0.])
    assert reward > -.01


def test_remaining_distance_is_continuous_between_route_vertices():
    env = KennyEnv(config=EnvConfig(stage="empty", domain_randomization=False))
    env.reset(seed=3)
    env.route = np.array([[0., 0.], [1., 0.], [2., 0.]])
    values = []
    for x in np.linspace(.1, 1.9, 19):
        env.estimate[:2] = [x, 0.]
        values.append(env._remaining())
    np.testing.assert_allclose(values, 2-np.linspace(.1, 1.9, 19))
    assert np.all(np.diff(values) < 0)


def test_arrival_shaping_prefers_stopping_near_goal():
    config = EnvConfig(stage="empty", max_steps=3, domain_randomization=False,
                       dropout=0., marker_dropout=0., sensor_noise=0.)
    stopped = KennyEnv(config=config)
    stopped.reset(seed=4)
    stopped.world.goal = stopped.estimate[:2] + [.30, 0.]
    stopped.route = np.array([stopped.estimate[:2], stopped.world.goal])
    stopped.previous_remaining = stopped._remaining()
    _, stop_reward, _, _, _ = stopped.step([-1., 0.])

    moving = KennyEnv(config=config)
    moving.reset(seed=4)
    moving.world.goal = moving.estimate[:2] + [.30, 0.]
    moving.route = np.array([moving.estimate[:2], moving.world.goal])
    moving.previous_remaining = moving._remaining()
    moving.velocity[:] = [.20, .30]
    moving.measured_velocity[:] = moving.velocity
    _, moving_reward, _, _, _ = moving.step([1., 1.])
    assert stop_reward > moving_reward
