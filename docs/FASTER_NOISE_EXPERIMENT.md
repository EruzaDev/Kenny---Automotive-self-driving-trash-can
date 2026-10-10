# Faster navigation experiments

These experiments create separate 0.60, 0.80, and 1.00 m/s models for simulation.
Changing maximum speed changes the saved robot/action contract; the 0.30 m/s
checkpoint cannot be directly resumed. Use `--initialize-baseline` to copy its
learned policy weights into a new PPO instance. This accepts only speed-cap and
adaptive-controller differences; sensors, features, normalization, timing and
robot dynamics must match. Optimizer state and step counters start fresh.
The existing models and deployment settings are preserved.

The faster models enable an explicit adaptive cruise controller: accelerate
along an aligned, clear planned route; slow before bends, the goal, and sensed
obstacles; brake for inadequate clearance or coverage. Cruise boosts apply only
when the policy requests forward motion; explicit stops and recovery speed
commands are preserved. Existing acceleration bounds ramp the speed gradually.
The hard sensor/cliff guard remains downstream of this controller.

Clearance includes conservative braking, worst configured command delay,
LiDAR acquisition delay, control time, noise, and localization uncertainty.
In dynamic worlds, sensor returns are treated conservatively as potentially
approaching/crossing people at up to the simulator's 0.8 m/s pedestrian ceiling.
This includes lateral entry during reaction and braking; there is no person
classification or tracking. Crowded or narrow spaces can therefore remain slow.
An obstacle appearing inside braking distance can still cause a collision.

From the simulator repository, run the bounded CPU pilot:

```sh
.venv/bin/python scripts/train_faster_noise.py --pilot --episodes 5 --workers 2 \
  --initialize-baseline \
  --output runs/faster-noise-adaptive-pilot
```

Each pilot has 8,192 PPO transitions (plus demonstration collection and
validation). It bootstraps from two successful noise-aware route-follower
demonstrations in empty worlds when cold-started. With `--initialize-baseline`,
it skips cloning and starts from the reviewed navigation policy. It then trains
in dynamic worlds with people,
noise, and a 20% simulation-only unshielded episode fraction. Pilot validation
uses two episodes per mode at startup and after the final PPO update.
An early pilot checks whether the pipeline works; it cannot establish that
the policy has converged. The default is sequential; `--workers` allows at most
three single-threaded training processes and at most two evaluation processes.
`--resume-experiment` adopts completed or active jobs in the same output directory
without overwriting their configurations or checkpoints. Interrupted trainers
require an explicit checkpoint resume through `kenny_rl.train` into a new run.

For a larger first training budget with 30 held-out episodes per condition:

```sh
.venv/bin/python scripts/train_faster_noise.py --episodes 30 \
  --initialize-baseline \
  --output runs/faster-noise-adaptive-100k
```

This uses 100,000 PPO transitions per speed, 20 successful bootstrap episodes,
and separate guarded/unshielded validation. That budget is an initial experiment,
not a convergence claim. `--steps` changes it. `--speeds 0.6` restricts the sweep.
Use a new output directory on every invocation. Repeat training with independent
`--seed` values and evaluate new held-out seeds with `--test-seed`.

The corrected controller uses contract `route-sensor-cruise-v2`. It separates
strict straight-route cruise eligibility from ordinary steering, using a smooth
heading cap so grid offsets do not force a stop. Crossing-corridor width shrinks
with the candidate speed, and pedestrian clearance retains lateral separation.
The original v1 cold-start pilots are incompatible with this controller and
are excluded from the editor model inventory. They remain on disk for audit.
Map walls are still treated as potentially approaching returns; uncertainty
and tight spaces can stop the robot. This controller does not classify people.

## Noise assumptions

All numbers below are provisional stress assumptions, not measurements of Kenny.

| Source | Training/test | Stress test |
| --- | --- | --- |
| Base LiDAR/depth range standard deviation | 0.03 m | 0.05 m |
| Added range standard deviation per m/s | 0.02 m | 0.04 m |
| Added range standard deviation per rad/s | 0.01 m | 0.02 m |
| Independent missing sensor samples | 3% | 10% (5% config, doubled by stress split) |
| Base marker dropout | 25% | 35% |
| Added marker dropout per m/s | 15 percentage points | 25 percentage points |
| Added marker dropout per rad/s | 10 percentage points | 15 percentage points |
| Wheel motor gains | 0.85–1.15 | 0.80–1.20 |
| Translation/rotation slip factors | 0.90–1.03 | 0.85–1.05 |
| Acceleration scale | 0.70–1.15 | 0.70–1.15 |
| Command delay | 0–0.3 s | 0–0.5 s |
| Whole sensor outage trigger probability | 1% per acquisition | 2% per acquisition |
| Whole outage length | 2–8 acquisition cycles | 3–10 acquisition cycles |

Motion terms depend on measured simulated velocity, not the configured speed cap:

```text
range_sigma = base_sigma + linear_coefficient * abs(v) + turn_coefficient * abs(omega)
marker_dropout = base_dropout + linear_coefficient * abs(v) + turn_coefficient * abs(omega)
```

The added marker dropout is capped at 95% unless the configured base is higher.
The simulator's new motion coefficients default to zero, preserving existing
configurations and checkpoint contracts. In-place turning can add motion error;
standing still retains base error. Localization uncertainty already grows with
distance, and sensor/command latency becomes more consequential at higher speed.

Increasing Gaussian range error and marker dropout is an approximation of
motion-related measurement degradation. It does not implement actual LiDAR
scan distortion, image rendering, motion blur, exposure control, electrical
noise, encoder errors, load-dependent tipping, or sensor calibration faults.
The simulated measured velocity itself is exact. Calibrate assumptions against
logged hardware measurements before using them for deployment decisions.

## Outputs and comparisons

Each speed directory contains its own `config.json`, `contract.json`,
`best_guarded.zip`, `best.zip`, and `final.zip` after successful completion.
The editor discovers those model ZIP files automatically; refresh its model list.
Choose `best_guarded.zip` for shielded map tests.

`experiment.json` records job commands/status. Per-speed logs contain bootstrap
and PPO diagnostics. `comparison.json` and the full per-condition reports compare
the existing seed-3 guarded 0.30 m/s policy with each faster pilot using the same
world seeds, noise rules, 21×21 room settings, and shielded testing. The baseline
retains its checkpoint controller; the faster models include adaptive cruise,
so this compares complete systems rather than isolating learned-policy gains.
The motion-related noise rules are shared, while each policy experiences noise
according to its own velocity. Test and stress splits use separate random streams.
Reports include success, collision, cliff, timeout, intervention fractions,
successful-episode steps, observed peak speed, and moving-speed percentiles.
Peak speed can exceed the command cap because motor gain and slip are randomized.

The comparison uses the dynamic stage, including moving people. This does not
qualify cliff behavior. Cliff tests, rectangular layouts, multiple training seeds,
and larger held-out sets require separate follow-up evaluations. Recovery/marker
sweeps are disabled in the comparison to measure checkpoint behavior consistently;
editor recovery tests can be performed separately.

If a faster model cannot reach goals or improves speed at the expense of collisions,
retain the 0.30 m/s model. None of these pilots authorizes a hardware speed change.
