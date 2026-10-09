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
    env.measured_velocity[:] = [.08, .6]
    env.down_hazard[:] = False
    env.down_valid[:] = True
    target, stopped = env._guard(np.array([.15, .6]))
    assert stopped
    assert 'turning_fast' in env.guard_reasons
    np.testing.assert_array_equal(target, [0., 0.])
    env.close()


def test_observed_clearance_caps_speed_without_blanket_turning_stop():
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
                                    domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=1)
    for scan in (env.lidar, env.depth, env.floor):
        scan.hits[:] = False
        scan.valid[:] = True
    env.down_hazard[:] = False
    env.down_valid[:] = True
    env.uncertainty = .02
    env.depth_memory = {}
    target, stopped = env._guard(np.array([.3, .3]))
    np.testing.assert_array_equal(target, [.3, .3])
    assert not stopped
    point = env.estimate[:2] + .4*np.array([np.cos(env.estimate[2]), np.sin(env.estimate[2])])
    env.depth_memory = {(0, 0): (point, env.steps)}
    target, limited = env._guard(np.array([.3, .3]))
    assert limited and 0 < target[0] < .3
    assert target[1] == .3
    # The limit belongs to the shield. Its simulation ablation must still
    # execute the original request, and a new episode cannot inherit points.
    env.shield_active = False
    target, stopped = env._guard(np.array([.3, .3]))
    np.testing.assert_array_equal(target, [.3, .3])
    assert not stopped
    env.reset(seed=1)
    assert (0, 0) not in env.depth_memory
    env.close()


def test_route_recovery_brakes_scans_then_uses_estimated_route():
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
                                   route_recovery_enabled=True, domain_randomization=False))
    env.reset(seed=1)
    env.measured_velocity[:] = [.08, 0.]
    np.testing.assert_array_equal(env._route_recovery_target(), [0., 0.])
    env.measured_velocity[:] = 0.
    assert env._route_recovery_target()[1] > 0
    direction = np.array([np.cos(env.estimate[2]), np.sin(env.estimate[2])])
    env.route = np.array([env.estimate[:2], env.estimate[:2]+direction])
    env.route_recovery_ticks = 20
    target = env._route_recovery_target()
    assert 0 < target[0] <= .12
    assert abs(target[1]) < 1e-8
    env.measured_velocity[0] = .08
    assert env._route_recovery_target()[0] > 0  # do not brake every moving tick
    env.uncertainty = .36
    assert env._route_recovery_target()[0] == 0
    env.measured_velocity[:] = 0.
    assert env._route_recovery_target()[0] == 0
    env.route = np.empty((0, 2))
    env.uncertainty = .02
    assert env._route_recovery_target()[0] == 0
    env.route = np.array([env.estimate[:2], env.estimate[:2]+.1*direction])
    env.world.goal = env.estimate[:2]+.1*direction
    np.testing.assert_array_equal(env._route_recovery_target(), [0., 0.])
    env.reset(seed=1)
    assert not env.route_recovery_active and env.route_recovery_ticks == 0
    env.close()


def test_route_recovery_cannot_bypass_floor_or_localization_guards():
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
                                   route_recovery_enabled=True, domain_randomization=False))
    env.reset(seed=1)
    env.uncertainty = .36
    target, stopped = env._guard(np.array([.12, 0.]))
    assert stopped and target[0] == 0
    assert 'localization_uncertain' in env.guard_reasons
    env.uncertainty = .02
    env.down_valid[:] = False
    target, stopped = env._guard(np.array([0., .3]))
    assert stopped
    np.testing.assert_array_equal(target, [0., 0.])
    env.close()
