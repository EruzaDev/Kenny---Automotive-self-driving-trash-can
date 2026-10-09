import numpy as np

from kenny_rl.config import EnvConfig
from kenny_rl.env import KennyEnv


def test_recovery_rotates_without_translation_and_respects_downward_interlock():
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
        domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=1)
    env.stalled_steps = 30
    env.step([-1., 0.])
    assert env.recovery_steps == 1
    assert env.executed[0] == -1.
    assert env.executed[1] > 0.
    env.stalled_steps = 30
    env.down_hazard[:] = True
    env.step([-1., 0.])
    np.testing.assert_array_equal(env.executed, [-1., 0.])
    env.close()


def test_guard_brakes_before_turning_with_residual_translation():
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True))
    env.reset(seed=1)
    env.measured_velocity[:] = [.08, .24]
    env.down_hazard[:] = False
    env.down_valid[:] = True
    target, stopped = env._guard(np.array([.15, .24]))
    assert stopped
    assert 'turning_fast' in env.guard_reasons
    np.testing.assert_array_equal(target, [0., 0.])
    env.close()
