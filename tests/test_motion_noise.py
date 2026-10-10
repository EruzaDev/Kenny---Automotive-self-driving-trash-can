from dataclasses import replace
import math
from unittest.mock import patch

import numpy as np
import pytest

from kenny_rl.config import EnvConfig, RobotConfig
from kenny_rl.env import KennyEnv


def test_motion_noise_uses_actual_motion_and_handles_turning():
    config = EnvConfig(sensor_noise=.03, marker_dropout=.25,
                       motion_range_noise_per_mps=.02,
                       motion_range_noise_per_radps=.01,
                       motion_marker_dropout_per_mps=.15,
                       motion_marker_dropout_per_radps=.1)
    env = KennyEnv(RobotConfig(max_speed=1.), config)
    try:
        env.reset(seed=42)
        assert env._motion_sensor_errors() == pytest.approx((.03, .25))
        env.measured_velocity = np.array([1., .6])
        assert env._motion_sensor_errors() == pytest.approx((.056, .46))
        env.measured_velocity = np.array([0., -.6])
        assert env._motion_sensor_errors() == pytest.approx((.036, .31))
        env.measured_velocity = np.array([100., 100.])
        assert env._motion_sensor_errors()[1] == .95
    finally:
        env.close()


def test_motion_noise_reaches_lidar_depth_and_marker_detection():
    env = KennyEnv(config=EnvConfig(motion_range_noise_per_mps=.1,
                                   motion_marker_dropout_per_mps=.2))
    try:
        env.reset(seed=42)
        env.measured_velocity = np.array([.3, 0.])
        from kenny_rl import sensors
        with patch.object(sensors, "lidar", wraps=sensors.lidar) as lidar, \
             patch.object(sensors, "depth", wraps=sensors.depth) as depth:
            env._sense(force=True)
            assert lidar.call_args.args[4] == pytest.approx(.045)
            assert depth.call_args.args[4] == pytest.approx(.045)
        with patch.object(sensors, "marker_visible", wraps=sensors.marker_visible) as marker:
            env.step(np.array([1., 0.], dtype=np.float32))
            assert marker.call_args.args[4] == pytest.approx(env._motion_sensor_errors()[1])
    finally:
        env.close()


def test_default_motion_noise_preserves_sensor_sequences():
    baseline = KennyEnv()
    explicit = KennyEnv(config=replace(EnvConfig(), motion_range_noise_per_mps=0.,
                                      motion_range_noise_per_radps=0.,
                                      motion_marker_dropout_per_mps=0.,
                                      motion_marker_dropout_per_radps=0.))
    try:
        a, _ = baseline.reset(seed=123)
        b, _ = explicit.reset(seed=123)
        np.testing.assert_array_equal(a, b)
        for action in ([1., .2], [0., -.5], [-1., 1.]):
            a = baseline.step(np.asarray(action, dtype=np.float32))
            b = explicit.step(np.asarray(action, dtype=np.float32))
            np.testing.assert_array_equal(a[0], b[0])
            assert a[1:] == b[1:]
    finally:
        baseline.close()
        explicit.close()


@pytest.mark.parametrize("field", ["motion_range_noise_per_mps", "motion_range_noise_per_radps",
                                   "motion_marker_dropout_per_mps", "motion_marker_dropout_per_radps"])
@pytest.mark.parametrize("value", [-1., math.nan, math.inf, "bad", True])
def test_motion_noise_settings_reject_invalid_values(field, value):
    with pytest.raises(ValueError, match=field):
        EnvConfig(**{field: value})
