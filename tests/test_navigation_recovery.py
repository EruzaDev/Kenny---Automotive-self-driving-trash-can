from dataclasses import asdict, replace
import json

import numpy as np
import pytest

from kenny_rl.config import EnvConfig, load_config
from kenny_rl.env import KennyEnv
from kenny_rl.planner import GridPlanner


def test_clearance_cost_prefers_open_route_without_blocking_narrow_passages():
    planner = GridPlanner(8, .25, .16)
    planner.static[12:20, 12:20] = True
    start, goal = planner.point((6, 11)), planner.point((26, 11))
    direct = planner.path(start, goal)
    planner.clearance_weight = 4.
    wide = planner.path(start, goal)
    assert wide[:, 1].min() < direct[:, 1].min()
    assert all(not planner.static[planner.cell(p)] for p in wide)
    planner.static[:] = True
    planner.static[:, 11] = False
    assert len(planner.path(start, goal))  # soft cost must not seal narrow routes


def test_observing_known_wall_does_not_double_inflate_it():
    planner = GridPlanner(8, .25, .16)
    planner.set_walls([[3., 0., 0., 3.2, 3., 1.]])
    static = planner.static.copy()
    planner.observe([[3., 2.]], step=0)
    np.testing.assert_array_equal(planner.expires, 0)
    np.testing.assert_array_equal(planner.static, static)
    # Small range noise can place a return in the cell beside the mapped wall.
    wall_cell = np.argwhere(planner.static)[0]
    adjacent = np.minimum(wall_cell+[1, 0], planner.n-1)
    planner.observe([planner.point(adjacent)], step=0)
    np.testing.assert_array_equal(planner.expires, 0)


def test_observing_unknown_obstacle_still_adds_temporary_memory():
    planner = GridPlanner(8, .25, .16)
    planner.set_walls([[3., 0., 0., 3.2, 3., 1.]])
    planner.observe([[5., 5.]], step=7)
    assert planner.expires[planner.cell([5., 5.])] == 57


def test_observed_obstacle_inflation_uses_grid_cell_uncertainty():
    planner = GridPlanner(8, .25, .16)
    cell = np.asarray(planner.cell([5., 5.]))
    planner.observe([[5., 5.]], step=0)

    # The robot footprint plus half a cell diagonal blocks the four adjacent
    # cells. It must not add the old full-cell margin, which also blocked the
    # diagonal ring and could close otherwise traversable mixed-stage routes.
    for offset in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
        index = tuple(cell + offset)
        assert planner.expires[index] == 50
    for offset in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        index = tuple(cell + offset)
        assert planner.expires[index] == 0


def test_blocked_goal_cell_routes_to_free_point_inside_arrival_region():
    planner = GridPlanner(8, .25, .16)
    start = np.array([1.125, 3.125])
    goal = np.array([3.24, 3.125])
    planner.expires[planner.cell(goal)] = 100

    assert not len(planner.navigation_path(start, goal, step=1))
    route = planner.navigation_path(start, goal, step=1, goal_tolerance=.25)

    assert len(route)
    assert planner.navigation_mode == "goal_tolerance"
    assert np.linalg.norm(route[-1]-goal) < .25
    assert not planner.blocked(1)[planner.cell(route[-1])]


def test_goal_region_fallback_does_not_bypass_non_goal_blockage():
    planner = GridPlanner(8, .25, .16)
    start = np.array([1.125, 3.125])
    goal = np.array([6.125, 3.125])
    planner.static[12, :] = True

    route = planner.navigation_path(start, goal, step=1, goal_tolerance=.25)

    assert not len(route)
    assert planner.navigation_mode == "blocked"


def test_clearance_configuration_validates_and_keeps_policy_contract():
    for weight in (-1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            EnvConfig(route_clearance_weight=weight)
    r, original, _ = load_config("configs/server.json")
    r2, recovery, _ = load_config("configs/server_static_recovery.json")
    assert KennyEnv(r, original).contract() == KennyEnv(r2, recovery).contract()
    normalized = replace(recovery, stage=original.stage, route_clearance_weight=0.)
    assert json.loads(json.dumps(asdict(normalized))) == json.loads(json.dumps(asdict(original)))


def test_mixed_fine_grid_profile_preserves_policy_contract():
    robot, baseline, _ = load_config("configs/server_static_avoidance.json")
    fine_robot, fine, training = load_config("configs/server_mixed_finegrid.json")
    assert fine.stage == "mixed" and fine.grid_resolution == .125
    assert training["total_timesteps"] == 150000 and training["dual_validation"]
    assert KennyEnv(robot, baseline).contract() == KennyEnv(fine_robot, fine).contract()


def test_route_lookahead_is_independent_of_grid_resolution():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.estimate = np.array([.1, 0., 0.])

    env.route = np.column_stack((np.arange(0., 3.01, .25), np.zeros(13)))
    coarse, coarse_valid = env._lookahead()
    env.route = np.column_stack((np.arange(0., 3.01, .125), np.zeros(25)))
    fine, fine_valid = env._lookahead()

    np.testing.assert_allclose(coarse, [[.5, 0.], [1., 0.], [2., 0.]])
    np.testing.assert_allclose(fine, coarse)
    np.testing.assert_array_equal(coarse_valid, np.ones(3))
    np.testing.assert_array_equal(fine_valid, np.ones(3))


def test_route_lookahead_interpolates_and_clamps_to_goal():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.estimate = np.array([0., 0., 0.])
    env.route = np.array([[0., 0.], [.3, 0.], [.3, .4], [.3, .6]])

    points, valid = env._lookahead()

    np.testing.assert_allclose(points, [[.3, .2], [.3, .6], [.3, .6]])
    np.testing.assert_array_equal(valid, np.ones(3))


def clear_sensors(env):
    env.uncertainty = .02
    env.down_hazard[:] = False
    env.down_valid[:] = True
    for scan in (env.lidar, env.depth, env.floor):
        scan.valid[:] = True
        scan.hits[:] = False
        scan.ranges[:] = 6.


@pytest.mark.parametrize("reason", ["downward_hazard", "downward_invalid",
    "localization_uncertain", "lidar_invalid", "depth_invalid", "floor_invalid",
    "lidar_obstacle", "depth_obstacle", "floor_hazard"])
def test_guard_reasons_stop_motion_and_reset(reason):
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    clear_sensors(env)
    if reason == "downward_hazard":
        env.down_hazard[0] = True
    elif reason == "downward_invalid":
        env.down_valid[0] = False
    elif reason == "localization_uncertain":
        env.uncertainty = .4
    else:
        sensor, kind = reason.split("_")
        scan = getattr(env, sensor)
        if kind == "invalid":
            scan.valid[:] = False
        else:
            i = np.argmin(abs(scan.angles))
            scan.hits[i], scan.ranges[i] = True, .1
    target, stopped = env._guard(np.array([.3, .2]))
    assert stopped and reason in env.guard_reasons
    if reason in {"downward_hazard", "downward_invalid", "floor_hazard", "floor_invalid"}:
        np.testing.assert_array_equal(target, [0, 0])
    else:
        np.testing.assert_array_equal(target, [0, .2])
    clear_sensors(env)
    _, stopped = env._guard(np.array([.3, .2]))
    assert not stopped and env.guard_reasons == ()


def test_guard_stops_for_isolated_forward_lidar_dropout():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    clear_sensors(env)
    front = np.flatnonzero(np.abs(env.lidar.angles) < np.deg2rad(40))

    env.lidar.valid[front[len(front)//2]] = False
    target, stopped = env._guard(np.array([.3, 0.]))
    np.testing.assert_array_equal(target, [0., 0.])
    assert stopped and "lidar_invalid" in env.guard_reasons


def test_rotation_recovery_never_overrides_floor_or_downward_interlock():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    clear_sensors(env)
    env.lidar.valid[:] = False
    target, lidar_stopped = env._guard(np.array([.3, .25]))
    np.testing.assert_array_equal(target, [0., .25])
    assert lidar_stopped

    env.down_valid[0] = False
    target, downward_stopped = env._guard(np.array([.3, .25]))
    np.testing.assert_array_equal(target, [0., 0.])
    assert downward_stopped


def test_guard_turns_in_place_before_translating_through_tight_arc():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    clear_sensors(env)

    env.velocity[1] = .56
    target, stopped = env._guard(np.array([.2, .1]))

    np.testing.assert_array_equal(target, [0., .1])
    assert stopped and "turning_fast" in env.guard_reasons

    env.velocity[1] = .5
    target, stopped = env._guard(np.array([.2, .5]))
    np.testing.assert_array_equal(target, [.2, .5])
    assert not stopped and env.guard_reasons == ()


def test_depth_obstacle_gets_extra_stopping_buffer():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    clear_sensors(env)
    center = np.argmin(abs(env.depth.angles))
    env.depth.hits[center] = True
    env.depth.ranges[center] = .48

    target, stopped = env._guard(np.array([.3, 0.]))

    np.testing.assert_array_equal(target, [0., 0.])
    assert stopped and "depth_obstacle" in env.guard_reasons


def test_episode_diagnostics_reset_and_count_no_route_separately():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    env.route = np.empty((0, 2))
    _, _, _, _, info = env.step([1., 0.])
    assert info["no_route_steps"] == 1
    assert info["interventions"] == 0
    _, info = env.reset(seed=1)
    assert info["no_route_steps"] == 0 and info["intervention_reasons"] == {}
