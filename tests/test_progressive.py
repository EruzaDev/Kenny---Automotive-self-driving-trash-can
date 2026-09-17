import numpy as np

from kenny_rl.config import EnvConfig, load_config
from kenny_rl.env import KennyEnv
from kenny_rl.planner import GridPlanner
from kenny_rl.sensors import Scan


def test_invalid_scan_reveals_nothing_and_hits_occlude():
    planner = GridPlanner(10, .25, .16, progressive=True)
    scan = Scan(np.array([2.]), np.array([False]), np.array([0.]), np.array([True]))
    planner.observe_scan([1.125, 1.125], 0., scan)
    assert not planner.known.any()
    scan.valid[:] = True
    planner.observe_scan([1.125, 1.125], 0., scan)
    assert planner.known[planner.cell([2., 1.125])]
    assert planner.occupied[planner.cell([3.125, 1.125])]
    assert not planner.known[planner.cell([4., 1.125])]
    assert planner.blocked(100000)[planner.cell([3.125, 1.125])]


def test_frontier_then_exact_goal_after_map_reveal():
    planner = GridPlanner(10, .25, .16, progressive=True)
    planner.known[:12, :12] = True
    start, goal = np.array([1., 1.]), np.array([8.1, 8.2])
    route = planner.navigation_path(start, goal)
    assert len(route) and planner.navigation_mode == "explore"
    assert all(planner.known[planner.cell(p)] for p in route)
    assert not np.allclose(route[-1], goal)
    planner.known[:] = True
    route = planner.navigation_path(start, goal)
    assert planner.navigation_mode == "goal"
    np.testing.assert_allclose(route[-1], goal)


def test_frontier_is_reachable_and_no_diagonal_corner_cutting():
    planner = GridPlanner(10, .25, .16, progressive=True)
    planner.known[4, 4] = True
    planner.known[5, 5] = True
    route = planner.navigation_path(planner.point((4, 4)), planner.point((5, 5)))
    assert not len(route)
    assert planner.navigation_mode == "blocked"


def test_progressive_env_does_not_preload_walls_and_reproduces():
    env = KennyEnv(config=EnvConfig(stage="static", map_mode="progressive", max_steps=5))
    first, info = env.reset(seed=8)
    assert not env.planner.static.any()
    assert 0 < info["mapped_fraction"] < 1
    transition = env.step(np.array([-.5, .2]))
    again, _ = env.reset(seed=8)
    np.testing.assert_array_equal(first, again)
    repeated = env.step(np.array([-.5, .2]))
    np.testing.assert_array_equal(transition[0], repeated[0])
    assert transition[1:] == repeated[1:]


def test_progressive_finetune_preserves_policy_contract():
    robot, known, _ = load_config("configs/server.json")
    other_robot, progressive, _ = load_config("configs/server_progressive.json")
    assert KennyEnv(robot, known).contract() == KennyEnv(other_robot, progressive).contract()


def test_arrival_profile_slows_without_crawling_outside_goal_radius():
    env = KennyEnv()
    assert env.approach_speed(.4) == env.robot.max_speed
    assert env.approach_speed(.26) > .1
    assert 0 < env.approach_speed(.21) < env.approach_speed(.26)
    assert env.approach_speed(.19) == 0


def test_stationary_robot_outside_arrival_does_not_earn_lingering_reward():
    env = KennyEnv(config=EnvConfig(stage="empty", domain_randomization=False,
                                   sensor_noise=0, dropout=0, marker_dropout=0))
    env.reset(seed=0, options={"start": [2., 2.], "goal": [2.3, 2.], "heading": 0.})
    _, reward, terminated, _, _ = env.step(np.array([-1., 0.]))
    assert not terminated
    assert reward < 0
