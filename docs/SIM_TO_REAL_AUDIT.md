# Training audit and four-GPU headless operation

Audited 2026-09-17. Verdict: suitable for controlled simulation experiments;
real-world transfer is **not validated**. No completed navigation qualification
or measurements from the assembled robot were supplied. Passing software tests
and completing PPO updates do not establish real-world performance.

## Parameters and disturbances

| Area | Current settings | Assessment |
| --- | --- | --- |
| Geometry | 0.23 × 0.23 × 0.485 m body, 0.235 m track, 0.04 m wheels | Provisional. Include wheels/bin protrusions and confirm track is wheel-center spacing. |
| Motion | 0.30 m/s, 0.60 rad/s, 0.50 m/s², 1.50 rad/s² | Command limits, not measured stopping performance. Random motor gain/slip can make true motion exceed nominal commands. |
| Control | 0.1 s steps, four observation frames, 1,076 features | Real inference/preprocessing must match timing, order, validity bits and scaling. |
| PPO | 128×128 actor/critic, learning rate 3e-4, gamma .99, GAE .95, clip .2, entropy .005 | Reasonable starting experiment, not tuned or proven to converge. Gamma gives about a 10 s discount horizon; long routes rely on intermediate progress reward. |
| Server rollouts | Eight workers × 512 steps = 4,096 transitions/run, batch 512, 10 epochs | Divisible minibatches; launcher adjusts workers to CPU allocation. One million transitions is a budget, not a success criterion. |
| PPO update guard | target KL .02 | Stops an overly large PPO update early; does not guarantee navigation safety. |
| Motor imbalance | Independent left/right gain 0.9–1.1 per episode | Existing disturbance; now configurable. Not a calibrated motor/battery model. |
| Slip | Linear/angular factors 0.94–1.02 per episode | Existing disturbance; now configurable. Does not model sudden patches, ramps or impacts. |
| Payload/response surrogate | Acceleration factor 0.7–1.1 per episode | Braking lower bound 0.35 m/s² in this model. Actual mass, inertia, center of gravity and tip-over are absent. |
| Command delay | Uniform 0–2 steps (0–200 ms) per episode | Configurable; not variable network jitter or MCU faults. |
| Range sensing | 0.015 m noise, 1.5% independent invalid samples (3% in stress) | Already present; simplified ray model, no material/lighting simulation. |
| Correlated sensor loss | Server: 0.5% burst-start probability per available acquisition; 2–5 invalid acquisitions | Added. LiDAR bursts span 0.4–1.0 s at 5 Hz; depth/floor bursts span 0.2–0.5 s at 10 Hz. Samples are invalid, not stale data; real disconnect/freshness handling remains separate. |
| Markers/localization | 15% missed detections, occlusion/FOV, noisy pose corrections | Existing surrogate. No real ArUco detector, false-ID rejection or calibrated covariance filter. |
| Scene disturbances | Moving people 0.15–0.8 m/s; bags moved/placed every 20 s in dynamic/full | Existing. Does not cover realistic crowds, contact physics or unpredictable fast motion. |

All configurable disturbance values are serialized in each run's `config.json`.
These are provisional ranges; measure both empty/full bin, low battery, floor
surfaces and simultaneous sensor load before narrowing or widening them.
Existing noise/marker dropout remains active when `domain_randomization=false`;
set noise/dropout/outage fields to zero for a noiseless diagnostic.

The guard previously read the exact hidden acceleration factor of each episode.
It now uses a configured worst-case bound instead. The contract is bumped to
`kenny-geometric-v3` and records the control period and guard braking bound.
Version-1 and v2 checkpoints are intentionally rejected by resume/evaluation:
start new server runs to evaluate the changed guard and reward consistently.
Feature dimensions remain unchanged, but old success statistics do not qualify
the new controller. V3 reduces the action-change penalty from 0.05 to 0.002 and
penalizes timeout by 5; v2 converged on a stationary, zero-success policy across
four seeds and 500 held-out episodes. V3 also projects position onto route
segments for continuous distance-to-go; the previous nearest-vertex calculation
incorrectly penalized correct travel through the first half of each route edge.
The final-approach term now penalizes deviation from a distance-based braking
speed, replacing the recurring positive slow-motion bonus that could reward
lingering. Success still requires low linear and angular speed. Routes terminate
at the exact goal rather than a cell center. These changes require re-evaluation;
they preserve the v3 policy input/output contract for fine-tuning.
The clean bootstrap collects successful demonstrations from the deterministic
route follower and fits the initial PPO actor to its normalized actions. The
follower uses estimated route lookahead and goal distance already represented in
the policy input; it receives no simulator truth. PPO exploration and validation
still determine whether the learned policy generalizes beyond that initialization.
Invalid range samples now carry the fixed maximum-range placeholder with validity
zero; previously they could retain a distance computed from hidden geometry even
though marked invalid. Whole-sensor outages use the same convention. The physical
adapter must reproduce this placeholder and validity pair; maximum range with
validity zero means unknown, never observed free space.

## Transfer gaps that training cannot resolve alone

- **Cliff stopping:** assumed forward lookahead is only 0.12 m beyond the body.
  At 0.30 m/s and 0.35 m/s² braking, `v*0.1 + v²/(2*a)` is about 0.159 m
  even before margins, extra latency, motor gain and slip (roughly 0.21 m with
  a 0.05 m margin). This already exceeds the lookahead. Measure the actual
  sensor-to-wheel geometry and stopping distance; lower speed or extend coverage.
  Sparse forward depth rays and point downward sensors cannot establish cliff safety.
- The circular footprint is conservative for the supplied body, but lateral/rear
  coverage, rotating near an edge, protrusions and the caster need hardware checks.
- Default known-map mode assumes a correct structural map. Optional progressive
  mode adds sensor-based mapping and frontier routing, not real scan matching or
  loop closure. See [progressive mapping](PROGRESSIVE_MAPPING.md). Mapping errors, unsurveyed
  starts, wrong markers and localization jumps need explicit tests.
- Depth age is synchronous in simulation. Delayed/frozen-but-plausible frames,
  timestamp/TF faults, camera extrinsic errors, glass, dark/reflective floors,
  sunlight, narrow chair legs and USB disconnects remain unqualified.
- Motor deadband, encoder quantization/faults, nonlinear braking, load movement,
  terrain changes and battery sag are only partially or not represented.
- ROS observation/action adapters, real motor PID, independent watchdog/stop
  chain, and hardware cliff sensors still need implementation and validation.

No invented calibration numbers were substituted for these missing measurements.
The practical sequence remains measured robot/sensor characterization, matching
Gazebo/ROS processing, shadow inference, then supervised low-speed trials.

## Four GPUs without a GUI

Use one allocated server job with four visible GPUs. Run `nvidia-smi` to record
the exact GPU names and driver: “RTX A5000” and “RTX 5000 Ada” are different names;
the launcher simply validates the CUDA devices exposed by the allocation.
The default is four independent seeds, **not distributed updates of one policy**.
Eight workers plus one learner per GPU needs 36 CPU slots. On 32 slots the launcher
chooses seven workers/run. Use `--cpu-budget` if affinity does not describe your
allocation; it also respects `SLURM_CPUS_PER_TASK`. RAM and throughput still need
measurement. Reducing worker count also changes rollout size.

Install on a Linux server using Python 3.10–3.12 in a fresh environment, with a
driver compatible with the chosen CUDA wheel. The pinned install follows the
[official PyTorch 2.5.1 CUDA 12.4 instructions](https://docs.pytorch.org/get-started/previous-versions/).
No desktop or Tk package is required. If `/tmp` has a small quota, use a writable
disk-backed scratch directory for `TMPDIR` during installation.

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --no-cache-dir torch==2.5.1 --index-url https://download.pytorch.org/whl/cu124
python -m pip install --no-cache-dir -e '.[train,test]'
python -m pytest -q

# Preview only: no GPU required and no output directory created.
python scripts/server_sweep.py --output runs/server-empty-v2 --stage empty --dry-run

# Four GPUs, four seeds, CPU workers chosen from the allocation.
python scripts/server_sweep.py --output runs/server-empty-v2 --stage empty --steps 1000000
```

The launcher rejects unavailable GPUs before creating its output, preserves
`CUDA_VISIBLE_DEVICES`, records GPU names and commands in `sweep.json`, and saves
unbuffered `seed_N.log` files. It sets `MPLBACKEND=Agg` and one BLAS/OpenMP thread
per process; training never renders. CUDA errors do not silently fall back to CPU.
Single-process `kenny_rl.train --config configs/server.json` only uses one GPU;
use the sweep launcher for all four.

```bash
tail -f runs/server-empty-v2/seed_0.log
tensorboard --logdir runs/server-empty-v2 --host 127.0.0.1
```

For remote TensorBoard, forward port 6006 over SSH. Validation success, collision,
cliff, timeout and intervention rates are logged alongside PPO statistics.
The small MLP and CPU simulator may underutilize these GPUs; compare elapsed
transitions/second against CPU runs before spending a long allocation. This is
consistent with [SB3's PPO guidance](https://stable-baselines3.readthedocs.io/en/v2.5.0/modules/ppo.html).

## Evaluation and curriculum

Use validation to decide when to advance, rather than automatically promoting a
completed budget. Re-test earlier stages for forgetting. After empty-stage
validation passes, continue each seed from its own best checkpoint:

```bash
python scripts/server_sweep.py --output runs/server-static-v2 --stage static \
  --resume-from runs/server-empty-v2 --steps 1000000
```

Repeat deliberately through mixed, dynamic, cliffs and full. `--steps` is an
additional per-seed budget on resume; episode streams restart from the seed, so
this is not exact mid-episode replay. Use a new output path for every sweep.
For deployment qualification, evaluate frozen selected models on both test and
stress splits with separate reports per stage and seed. Example after full-stage
training (this example is one of the four seeds):

```bash
python -m kenny_rl.evaluate --model runs/server-full-v2/seed_0/best.zip \
  --stage full --split test --episodes 500 --output artifacts/seed0-full-test.json
python -m kenny_rl.evaluate --model runs/server-full-v2/seed_0/best.zip \
  --stage full --split stress --episodes 500 --output artifacts/seed0-full-stress.json
```

Report success, collisions, cliffs, timeouts and intervention rates for **all**
seeds, including failures. Unshielded evaluations belong in simulation only.
The proposed 95% success/zero observed contacts target in the robot guide is a
project target, not a proven threshold for real-world reliability. Final held-out
tests must not be repeatedly used to tune parameters. Real transfer requires
the hardware evidence described above even if all simulated episodes pass.

## Verification performed for this change

- 33 automated tests passed, including observation validity masking, seeded
  disturbances, outage recovery, guard independence from hidden braking, worker
  budgeting, and four-device dry-run command generation.
- Two short CPU seeds completed headlessly using two spawned workers each, saved
  checkpoints and validation metrics, and resumed into separate static-stage runs.
- A saved resumed model completed stress evaluation. These deliberately tiny
  runs timed out and demonstrate plumbing only, not learned navigation.
- On the CPU-only development installation, the GPU preflight failed with a
  clear error and created no sweep directory. Four-GPU execution, VRAM/RAM use
  and throughput have **not** been measured on the target server.
