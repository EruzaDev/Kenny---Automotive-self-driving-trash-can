"""Replay dimension overrides must preserve the saved checkpoint config."""
import sys

import pytest

from kenny_rl.config import EnvConfig, RobotConfig
from scripts import review_recovery


@pytest.mark.parametrize('value', ['4', 'nan', 'inf'])
def test_invalid_replay_dimensions_rejected_before_model_load(monkeypatch, tmp_path, value):
    monkeypatch.setattr(sys, 'argv', [
        'review_recovery.py', '--model', 'unused/best.zip',
        '--output', str(tmp_path / 'replay.json'), '--world-width', value])
    monkeypatch.setattr(review_recovery, 'load_config',
                        lambda _: (RobotConfig(), EnvConfig(), {}))
    def unexpected_load(*args, **kwargs):
        pytest.fail('Invalid dimensions must be rejected before loading the model')
    monkeypatch.setattr(review_recovery.PPO, 'load', unexpected_load)
    with pytest.raises(SystemExit) as error:
        review_recovery.main()
    assert error.value.code == 2
    assert not (tmp_path / 'replay.json').exists()


def test_rectangular_override_preserves_other_settings(monkeypatch, tmp_path):
    original = EnvConfig(world_width=21., world_height=21., stage='mixed',
                         max_steps=1800, route_clearance_weight=4.)
    monkeypatch.setattr(sys, 'argv', [
        'review_recovery.py', '--model', 'unused/best.zip',
        '--output', str(tmp_path / 'replay.json'), '--recovery-only',
        '--marker-sweep', '--world-width', '21', '--world-height', '10'])
    monkeypatch.setattr(review_recovery, 'load_config',
                        lambda _: (RobotConfig(), original, {}))
    monkeypatch.setattr(review_recovery.PPO, 'load', lambda *a, **k: object())
    class ConfigurationChecked(Exception):
        pass
    def check_env(robot, config):
        assert config.world_width == 21.
        assert config.world_height == 10.
        assert config.max_steps == original.max_steps
        assert config.domain_randomization == original.domain_randomization
        assert config.route_clearance_weight == original.route_clearance_weight
        assert config.marker_sweep_enabled
        assert config.split == 'test'
        raise ConfigurationChecked
    monkeypatch.setattr(review_recovery, 'KennyEnv', check_env)
    with pytest.raises(ConfigurationChecked):
        review_recovery.main()
    assert original.world_height == 21.
