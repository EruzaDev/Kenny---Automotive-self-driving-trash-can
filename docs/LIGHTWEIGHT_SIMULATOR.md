# Lightweight simulator: implementation and validation

This document describes the implemented environment, rather than the more ambitious Gazebo/ROS design in the original guide. The first implementation targets low RAM use and repeatable learning experiments. Dependencies and commands are in the [README](../README.md).

## Runtime design

`KennyEnv` implements the [Gymnasium environment contract](https://gymnasium.farama.org/api/env/): seeded reset; continuous two-component actions; observations, reward, termination, truncation and diagnostic info. It is importable without PyTorch, ROS or matplotlib.

- `world.py`: scene generation, reachable start/goal sampling, people, and clutter changes.
- `geometry.py`: exact ray/axis-aligned-box intersections and conservative circular collision checks.
- `sensors.py`: horizontal LiDAR, sampled 3D depth, visible floor evidence, downward sensors and marker visibility.
- `planner.py`: inflated occupancy-grid A* with diagonal corner cutting disabled.
- `env.py`: noisy localization, motor response, sensor-based guard, reward and stacked observations.
- `train.py`, `evaluate.py`: PPO, checkpointing, seeded evaluation and portable configuration bundles.
- `scripts/server_sweep.py`: bounded independent experiments on CPU or assigned CUDA devices.

The body footprint is a circle of radius `hypot(0.23, 0.23)/2 = 0.1626 m`. This encloses the square body at every heading, so turns are conservative; it can reject spaces the actual square could fit through. The collision check uses the body's 0.485 m height. All dimensions are meters.

The renderer is only a view. Collision checks are independent of pixels and run in substeps, including moving people. A floor gap is a negative obstacle represented separately from solid boxes. Contact terminates an episode; there is no simulated falling dynamics.

## Sensor and localization assumptions

LiDAR uses 72 horizontal rays at the configured mount height. Depth uses 36 azimuths and 9 elevations against 3D boxes, with floor occlusion and a configured near blind zone. Returns outside the body's collision height are excluded from the policy obstacle features. Sparse rays can miss narrow features; this is one reason Gazebo and real sensor validation remain necessary.

Floor evidence checks expected ground intersections with 7 downward elevations per 12 azimuth sectors and honors solid-object occlusion. Missing depth is represented as invalid evidence, distinct from an observed drop. Three downward sensors check points ahead of the footprint, assuming added physical sensors. They are not an existing capability of the Astra/X3 Pro combination.

The initial robot pose is initialized from a surveyed start. Wheel odometry follows wheel motion while the physical pose experiences randomized slip. A visible, unoccluded floor marker causes a noisy pose correction. This emulates the output of a pose estimator; no image detector, printed dictionary, camera calibration solver or full covariance filter is implemented. Marker IDs themselves are never policy features.

Structural walls form the supplied global map. The planner receives additional obstacle endpoints projected using estimated pose. It has no access to hidden bag/overhang/person positions or hidden cliffs. Temporary obstacle memory lasts 5 seconds; observed floor drops have longer retention. The planner updates every second. If it cannot find a path, the controller stops and keeps trying to replan.

## Observation schema v2

Each frame has **269 float32 features**; four consecutive frames produce **1,076 features**. Reset repeats the initial observation across history, so zero padding cannot be mistaken for zero-distance obstacles.

| Feature group | Count | Fixed normalization |
| --- | --- | --- |
| LiDAR ranges / validity | 72 + 72 | Divide by configured maximum range; validity 0/1 |
| Depth collision ranges / validity | 36 + 36 | Divide by camera range; validity 0/1 |
| Floor-drop ranges / validity | 12 + 12 | Divide by camera range; validity 0/1 |
| Downward hazard / validity | 3 + 3 | Boolean 0/1 |
| Three local route points / validity | 6 + 3 | Base-relative x/y divided by 3 m |
| Goal relative x/y | 2 | Divide by known world extent |
| Encoder-derived v/omega | 2 | Divide by configured speed limits |
| Requested / guarded action | 2 + 2 | Already normalized to [-1,1] |
| Localization health | 3 | Uncertainty / 0.5 m, marker age / 30 s, validity |
| Sensor age and guard status | 3 | LiDAR age in seconds, depth age (zero for synchronous surrogate), guard active |

All features are clipped to [-1,1]. Use the validity bits when interpreting missing sensor data. Frame order and feature order are fixed in `FEATURES`; `contract.json` records them with the robot configuration, control period and conservative guard braking bound. V2 retains the feature dimensions but rejects v1 checkpoints after the guard change. No running observation normalizer is used. A ROS adapter must match this normalization exactly, including the world/map extent scale.

`info` contains privileged diagnostic error and episode labels, but those are not fed to PPO. Scene truth is used for procedural validity checks, simulated sensor generation, contacts and evaluation labels only. Structural wall geometry and the goal are deliberately known inputs, corresponding to a map and requested goal at deployment.

## Dynamics, actions and guard

The policy output is normalized as:

```text
v = (action[0] + 1) × 0.5 × 0.30 m/s
omega = action[1] × 0.60 rad/s
```

`[-1, 0]` requests a stop. Reverse is disabled. Body speeds are converted to differential wheel targets, limited by the nominal 333 RPM / 80 mm wheel specification, and passed through gain variation, acceleration limits, ground slip and a random 0–2 step command delay. These are parameter surrogates for the measured drive, not an encoder electrical model or real motor PID.

The guard checks range evidence, unknown forward sensing, floor support, downward hazards and localization uncertainty. Its stop margin includes robot radius, braking distance and sensor latency. It does not look ahead using simulator collision truth. Braking is acceleration limited, so a late stop can still collide; evaluation reports such failures.

The braking margin uses the configured minimum acceleration scale, rather than
the hidden episode-specific draw. Motor gain, slip, acceleration and command
delay ranges are configurable. The server profile additionally introduces
2–5 consecutive invalid acquisitions with a 0.005 burst-start probability;
depth and floor validity fail together. These are invalid-sample bursts, not
transport-delay or frozen-frame simulation. See the [audit](SIM_TO_REAL_AUDIT.md)
for the full parameter assessment and the insufficient cliff-lookahead assumption.

In this lightweight version the guard is a deterministic simulation function, not an independently running ROS node or MCU interlock. Complete source timeouts, sensor disconnect latching, rear/side cliff coverage and electrical emergency stops remain hardware/Gazebo work.

## Reward and termination

Per step:

```text
5 × route progress in meters, clipped to [-0.1, 0.1]
- 0.01 time cost
- 0.05 × squared change in normalized requested action
- 0.02 while the guard intervenes
- 0.10 when a new guard intervention begins
+ 20 once for arrival
- 50 on collision or cliff entry
```

Progress is computed on the existing path from odometry motion before applying marker corrections or replanning; the reference is then reset. This prevents a pose correction or shorter replacement path from creating artificial reward. Intervention penalties are kept smaller than contact penalties so repeated noisy stop events do not make an early collision attractive.

Success requires both true and estimated position within 0.25 m of the goal and small linear/angular velocity. This is a simulation label; a real arrival checker must use measured pose/uncertainty and a destination marker. Goal success, collision and cliff entry terminate. The time limit truncates. Calling `step` on a finished episode or passing NaN actions raises an error.

## Generalization tests

Training includes randomized geometry, starts/goals, headings, wheel gains, slip, response and command delay. Per-step sensor noise, validity dropouts, and marker misses perturb observations. People move with randomized direction changes and reverse at solid obstacles. In dynamic/full stages, a bag is moved or placed every 200 steps outside the robot's immediate area. This modifies geometry without updating the supplied map; sensors must discover it.

Validation and test RNG streams are separate from training. Stress worlds additionally enlarge room size and add U-shaped dead ends. These checks expose failures on unfamiliar geometry, but cannot prove generalization to glass, stairs, glossy/dark floors, real crowds, ramps or an entirely different sensor mount. Add those to Gazebo and physical evaluations using real calibration.

The stress topology is held out by generation logic; ordinary validation/test layouts share the training families with different samples. Marking a run `test` does not itself establish a novel real-world domain.

## Local checks

The initial laptop checks used Python 3.10 and CPU-only PyTorch 2.5.1. A 1,000-step headless full-world benchmark measured approximately **185 simulation steps/second** and **38 MiB peak process RSS**. A 256-transition PPO installation check with validation measured approximately **335 MiB peak RSS**, including the learner. A subsequent 1,024-transition run using the laptop profile in a full world measured about **170 training transitions/second** and **339 MiB peak RSS**. These are local samples, not guarantees for other CPUs, renderers, or worker counts. The benchmark does not include neural training; the PPO sample does.

All **21 functional tests** passed. Checkpoint save/resume, held-out stress evaluation, and a two-worker spawned PPO run were also exercised. A debug route follower reached all five sampled empty-room goals; that confirms those tasks are controllable, not that PPO has learned them.

The tests cover Gymnasium's checker, deterministic resets, bounded observations across all stages, height-specific sensor detection, marker field of view and occlusion, parallel ray cases, floor drop versus invalid depth, A* detours, memory expiry, time-limit behavior, interlocks, scenario changes and stress sampling. The test suite is small enough to run on the laptop.

The short training check is deliberately too small to learn navigation: its validation episodes timed out. Do not deploy the resulting smoke-test weights or interpret successful command execution as policy convergence. Full training, generalization qualification, A5000 benchmarking, Gazebo validation and physical deployment remain separate work.
