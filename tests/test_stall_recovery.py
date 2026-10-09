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


def test_failed_scan_expires_and_cannot_immediately_retrigger(monkeypatch):
    from kenny_rl import sensors
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
        route_recovery_enabled=True, domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=1)
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: False)
    env.uncertainty = .21
    env.step([-1., 0.])
    assert env.route_recovery_active and not env.route_recovery_scan_armed
    assert env.route_recovery_localization_only
    for _ in range(19):
        env.step([-1., 0.])
    assert not env.route_recovery_active
    assert env.route_recovery_ticks == 0
    assert env.uncertainty > .21  # no marker correction was manufactured
    retry = env.route_recovery_retry_step
    env.stalled_steps = 100
    env.step([-1., 0.])
    assert not env.route_recovery_active  # both triggers respect cooldown
    env.steps = retry
    env.stalled_steps = 0
    env.step([-1., 0.])
    assert not env.route_recovery_active  # stale uncertainty alone is not a retry
    env.stalled_steps = 100
    env.step([-1., 0.])
    assert env.route_recovery_active  # an actual stall can trigger after cooldown
    assert not env.route_recovery_localization_only
    env.uncertainty = .4
    for _ in range(79):
        env.step([-1., 0.])
    assert not env.route_recovery_active  # hard eight-second cap even above the guard limit
    target, stopped = env._guard(np.array([.12, 0.]))
    assert stopped and target[0] == 0 and 'localization_uncertain' in env.guard_reasons
    env.reset(seed=1)
    assert env.route_recovery_retry_step == 0 and env.route_recovery_scan_armed
    env.close()


def test_marker_rearms_scan_but_does_not_cancel_cooldown(monkeypatch):
    from kenny_rl import sensors
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
        route_recovery_enabled=True, domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=1)
    env.route_recovery_scan_armed = False
    env.route_recovery_retry_step = 100
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: True)
    env.step([-1., 0.])
    assert env.route_recovery_scan_armed and env.uncertainty == .02
    env.uncertainty = .21
    env.stalled_steps = 100
    env.step([-1., 0.])
    assert not env.route_recovery_active
    env.close()


def test_recovery_mode_is_json_serializable_after_numpy_uncertainty():
    import json
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
        route_recovery_enabled=True, domain_randomization=False))
    env.reset(seed=1)
    env.uncertainty = np.float64(.02)
    env.stalled_steps = 100
    env.step([-1., 0.])
    assert type(env.route_recovery_localization_only) is bool
    json.dumps({'localization_only': env.route_recovery_localization_only})
    env.close()


def test_marker_sweep_brakes_and_ends_on_measured_heading_coverage(monkeypatch):
    from kenny_rl import sensors
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
        route_recovery_enabled=True, marker_sweep_enabled=True,
        domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=1)
    env.route_recovery_localization_only = True
    env.measured_velocity[:] = [.08, 0.]
    np.testing.assert_array_equal(env._route_recovery_target(), [0., 0.])
    env.measured_velocity[:] = 0.
    env.route_recovery_ticks = 0
    env.uncertainty = .21
    for scan in (env.lidar, env.depth, env.floor):
        scan.valid[:] = True
        scan.hits[:] = False
    env.down_valid[:] = True
    env.down_hazard[:] = False
    monkeypatch.setattr(env, '_sense', lambda *args, **kwargs: None)
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: False)
    env.step([-1., 0.])
    assert env.route_recovery_active
    while env.route_recovery_active:
        env.step([-1., 0.])
        assert env.steps <= 200
    assert env.route_recovery_sweep_angle >= 2*np.pi
    assert env.steps < 200
    assert not env.route_recovery_scan_armed
    assert env.uncertainty > .21  # coverage is not a fabricated pose correction
    assert env.route_recovery_retry_step > env.steps
    env.reset(seed=1)
    assert env.route_recovery_sweep_angle == 0.
    env.close()


def test_marker_sweep_blocked_by_floor_still_has_hard_deadline(monkeypatch):
    from kenny_rl import sensors
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
        route_recovery_enabled=True, marker_sweep_enabled=True,
        domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=1)
    env.uncertainty = .4
    env.down_valid[:] = False
    monkeypatch.setattr(env, '_sense', lambda *args, **kwargs: None)
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: False)
    for _ in range(200):
        env.step([-1., 0.])
        np.testing.assert_array_equal(env.executed, [-1., 0.])
    assert not env.route_recovery_active
    assert env.route_recovery_sweep_angle == 0.
    assert env.uncertainty > .4
    env.close()


def test_marker_sweep_releases_immediately_on_real_marker_signal(monkeypatch):
    from kenny_rl import sensors
    env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
        route_recovery_enabled=True, marker_sweep_enabled=True,
        domain_randomization=False, dropout=0., sensor_noise=0.))
    env.reset(seed=1)
    env.uncertainty = .21
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: False)
    env.step([-1., 0.])
    assert env.route_recovery_active
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: True)
    env.step([-1., 0.])
    assert not env.route_recovery_active and env.uncertainty == .02
    assert env.route_recovery_scan_armed
    assert env.route_recovery_retry_step > env.steps
    env.close()


def test_marker_sweep_earlier_trigger_does_not_change_bounded_baseline(monkeypatch):
    from kenny_rl import sensors
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: False)
    for enabled in (False, True):
        env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
            route_recovery_enabled=True, marker_sweep_enabled=enabled,
            domain_randomization=False))
        env.reset(seed=1)
        env.uncertainty = .16
        env.step([-1., 0.])
        assert env.route_recovery_active == enabled
        env.close()


def test_route_alignment_does_not_consume_marker_sweep_opportunity(monkeypatch):
    from kenny_rl import sensors
    monkeypatch.setattr(sensors, 'marker_visible', lambda *args: False)
    for enabled in (False, True):
        env = KennyEnv(config=EnvConfig(stage='empty', recovery_enabled=True,
            route_recovery_enabled=True, marker_sweep_enabled=enabled,
            domain_randomization=False))
        env.reset(seed=1)
        env.uncertainty = .02
        env.stalled_steps = 100
        env.step([-1., 0.])
        assert env.route_recovery_active and not env.route_recovery_localization_only
        assert env.route_recovery_scan_armed == enabled
        env.close()


def test_marker_sweep_flag_requires_a_boolean():
    import pytest
    for value in ('false', 1, None):
        with pytest.raises(ValueError, match='marker_sweep_enabled must be a boolean'):
            EnvConfig(marker_sweep_enabled=value)
