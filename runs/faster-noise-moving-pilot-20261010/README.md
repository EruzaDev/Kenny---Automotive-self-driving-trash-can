# Faster Kenny simulation checkpoints

| Speed cap | Selected checkpoint |
| --- | --- |
| 0.60 m/s | [speed_060/best_guarded.zip](speed_060/best_guarded.zip) |
| 0.80 m/s | [speed_080/best_guarded.zip](speed_080/best_guarded.zip) |
| 1.00 m/s | [speed_100/best_guarded.zip](speed_100/best_guarded.zip) |

Keep each checkpoint beside its `config.json` and `contract.json`. Clone the
repository to obtain all three models; no separate model download is required.
Checksums are recorded in `sha256.json`. Training details and provisional noise
assumptions are in [the experiment guide](../../docs/FASTER_NOISE_EXPERIMENT.md).

These pilot runs initialized from the reviewed 0.30 m/s policy and performed
8,192 PPO transitions each. Guarded validation selected the initialized weights
over the final updates in all three runs. These are speed/controller variants
of the learned baseline, not demonstrated improvements from PPO fine-tuning.
`checkpoint-selection.json` records that choice, and the test/stress reports
include unsuccessful episodes. They are simulation experiments.

Example from the simulator repository:

```sh
.venv/bin/python -m kenny_rl.replay \
  --environment /path/to/environment.json \
  --model runs/faster-noise-moving-pilot-20261010/speed_100/best_guarded.zip \
  --controller marker-sweep --steps 1800 --seed 42 --output /tmp/replay.json
```

The saved controller uses conservative cruise (`route-sensor-cruise-v2`). The
Kenny map creator's **Relaxed cruise** preview is a separate simulation override;
it is not encoded in the policy ZIP and must be selected in that editor.
