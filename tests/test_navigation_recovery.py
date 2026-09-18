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


def test_clearance_configuration_validates_and_keeps_policy_contract():
    for weight in (-1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            EnvConfig(route_clearance_weight=weight)
    r, original, _ = load_config("configs/server.json")
    r2, recovery, _ = load_config("configs/server_static_recovery.json")
    assert KennyEnv(r, original).contract() == KennyEnv(r2, recovery).contract()
    normalized = replace(recovery, stage=original.stage, route_clearance_weight=0.)
    assert json.loads(json.dumps(asdict(normalized))) == json.loads(json.dumps(asdict(original)))


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
    np.testing.assert_array_equal(target, [0, 0])
    clear_sensors(env)
    _, stopped = env._guard(np.array([.3, .2]))
    assert not stopped and env.guard_reasons == ()


def test_guard_tolerates_isolated_lidar_dropout_but_stops_for_blind_sector():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    clear_sensors(env)
    front = np.flatnonzero(np.abs(env.lidar.angles) < np.deg2rad(40))

    env.lidar.valid[front[len(front)//2]] = False
    target, stopped = env._guard(np.array([.3, 0.]))
    np.testing.assert_array_equal(target, [.3, 0.])
    assert not stopped and "lidar_invalid" not in env.guard_reasons

    env.lidar.valid[:] = True
    middle = len(front)//2
    env.lidar.valid[front[middle-1:middle+2]] = False
    target, stopped = env._guard(np.array([.3, 0.]))
    np.testing.assert_array_equal(target, [0., 0.])
    assert stopped and "lidar_invalid" in env.guard_reasons


def test_lidar_coverage_rejects_many_scattered_invalid_rays():
    valid = np.ones(15, dtype=bool)
    valid[[0, 4, 8, 12]] = False
    assert KennyEnv._lidar_coverage_insufficient(valid)
    valid[:] = False
    assert KennyEnv._lidar_coverage_insufficient(valid)


def test_episode_diagnostics_reset_and_count_no_route_separately():
    env = KennyEnv(config=EnvConfig(stage="empty"))
    env.reset(seed=1)
    env.route = np.empty((0, 2))
    _, _, _, _, info = env.step([1., 0.])
    assert info["no_route_steps"] == 1
    assert info["interventions"] == 0
    _, info = env.reset(seed=1)
    assert info["no_route_steps"] == 0 and info["intervention_reasons"] == {}
