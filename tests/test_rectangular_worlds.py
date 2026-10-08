import numpy as np

from kenny_rl.config import EnvConfig
from kenny_rl.env import KennyEnv


def test_generated_21_by_10_world_uses_native_rectangular_bounds():
    env = KennyEnv(config=EnvConfig(stage="full", world_width=21., world_height=10.,
                                    domain_randomization=False, max_steps=10))
    observation, _ = env.reset(seed=3)
    assert env.observation_space.contains(observation)
    assert env.world.width == 21.
    assert env.world.height == 10.
    assert env.planner.shape == (84, 40)
    assert np.all(env.world.start < [21., 10.])
    assert np.all(env.world.goal < [21., 10.])


def test_rectangular_reset_options_reject_points_outside_short_axis():
    env = KennyEnv(config=EnvConfig(stage="empty", world_width=21., world_height=10.,
                                    domain_randomization=False))
    env.reset(seed=2, options={"start": [1., 1.], "goal": [20., 9.]})
    assert len(env.route)
    try:
        env.reset(seed=2, options={"start": [1., 10.1]})
    except ValueError as error:
        assert "collision-free" in str(error)
    else:
        raise AssertionError("point outside rectangular room was accepted")
