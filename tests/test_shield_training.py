from dataclasses import replace

import numpy as np
import pytest

from kenny_rl.config import EnvConfig, load_config
from kenny_rl.env import KennyEnv


@pytest.mark.parametrize("split", ["validation", "test", "stress"])
def test_evaluation_never_samples_training_shield_mixture(split):
    config = EnvConfig(stage="empty", split=split, train_unshielded_fraction=1.)
    guarded = KennyEnv(config=config)
    raw = KennyEnv(config=replace(config, shield=False))
    guarded.reset(seed=5)
    raw.reset(seed=5)
    assert guarded.shield_active and not raw.shield_active
    np.testing.assert_array_equal(guarded.world.start, raw.world.start)


def test_mixture_is_reproducible_and_does_not_change_scenarios():
    mixed = KennyEnv(config=EnvConfig(stage="empty", train_unshielded_fraction=.5))
    baseline = KennyEnv(config=EnvConfig(stage="empty"))
    modes = set()
    for seed in range(10):
        a, _ = mixed.reset(seed=seed)
        mode = mixed.shield_active
        modes.add(mode)
        b, _ = baseline.reset(seed=seed)
        np.testing.assert_array_equal(a, b)
        mixed.reset(seed=seed)
        assert mixed.shield_active == mode
    assert modes == {True, False}


def test_unshielded_guard_reports_shadow_reason_without_overriding():
    env = KennyEnv(config=EnvConfig(stage="empty", shield=False))
    env.reset(seed=2)
    env.down_hazard[:] = True
    target = np.array([.2, .1])
    actual, intervened = env._guard(target)
    np.testing.assert_array_equal(actual, target)
    assert not intervened and "downward_hazard" in env.guard_reasons


@pytest.mark.parametrize("shield", [True, False])
def test_penalty_distinguishes_obstacle_from_sensor_only(shield):
    config = EnvConfig(stage="empty", shield=shield, domain_randomization=False,
                       obstacle_command_penalty=.15, sensor_command_penalty=.005,
                       intervention_penalty=0., intervention_onset_penalty=0.)
    rewards = []
    for reason in ("lidar_obstacle", "lidar_invalid"):
        env = KennyEnv(config=config)
        env.reset(seed=0)
        def guard(target):
            env.guard_reasons = (reason,)
            return (np.zeros(2), True) if shield else (target, False)
        env._guard = guard
        _, reward, _, _, info = env.step([.1, 0.])
        rewards.append(reward)
        assert info["unsafe_command_steps"] == int(reason == "lidar_obstacle")
        assert info["interventions"] == int(shield)
        assert bool(info["intervention_reasons"]) == shield
    assert rewards[1]-rewards[0] == pytest.approx(.145)


def test_new_profile_preserves_contract_and_validates_settings():
    r, old, _ = load_config("configs/server_static_recovery.json")
    r2, new, training = load_config("configs/server_static_avoidance.json")
    assert KennyEnv(r, old).contract() == KennyEnv(r2, new).contract()
    assert training["dual_validation"]
    for value in (-1., 1.1, float("nan")):
        with pytest.raises(ValueError):
            EnvConfig(train_unshielded_fraction=value)
    for value in (-1., float("inf"), float("nan")):
        with pytest.raises(ValueError):
            EnvConfig(obstacle_command_penalty=value)


def test_distillation_profile_resumes_with_unshielded_expert():
    r, old, _ = load_config("configs/server_static_avoidance.json")
    r2, new, training = load_config("configs/server_static_distill.json")
    assert KennyEnv(r, old).contract() == KennyEnv(r2, new).contract()
    assert training["behavior_cloning_on_resume"]
    assert training["behavior_cloning_unshielded"]
    assert training["behavior_cloning_episodes"] == 200
