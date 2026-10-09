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
