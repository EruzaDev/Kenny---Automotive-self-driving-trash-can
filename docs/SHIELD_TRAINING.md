# Static avoidance training with reduced guard dependence

`configs/server_static_avoidance.json` is an experimental fine-tuning profile
for v3 checkpoints. It changes the training distribution and reward, not the
guard's thresholds or the policy's input/output contract.

- Approximately 80% of training episodes enforce the shield; 20% do not.
  Sampling is per episode, not per step, so the transition ratio may differ.
  A separate seeded RNG leaves world generation and sensor draws unchanged.
- Validation/test/stress modes never sample this mixture. `shield=true` always
  enforces the guard there; `--unshielded` explicitly disables enforcement.
- Guard logic still runs in unshielded mode to identify unsafe commands, but
  does not alter actuation. These shadow events are not counted as interventions.
- Commands flagged by obstacle or floor-hazard evidence cost 0.15 per step.
  Sensor-invalid/localization-only requests cost 0.005. If both apply, only the
  obstacle penalty applies. These replace the legacy generic intervention and
  onset costs in this profile. Other profiles retain their old reward settings.
- Collision/cliff termination and the existing -50 penalty remain active.
  The existing no-route stop remains active even during unshielded episodes.

This is simulation training only. The mixture does not authorize disabling any
physical watchdog, emergency stop, or cliff protection. High unshielded failure
can reflect changed dynamics, missing policy experience and localization/sensor
failures as well as weak avoidance; reward tuning alone is not a proven cure.
The coefficients and 20% mixture are initial experimental choices, not optimized
values or an established performance gain. Monitor for stationary-policy collapse.

## Continue existing training

After syncing the code to the HPC server, use a new output directory:

```bash
python scripts/server_sweep.py \
  --config configs/server_static_avoidance.json \
  --output runs/server-static-v3-avoidance \
  --resume-from runs/server-static-v3-recovery \
  --resume-checkpoint best.zip \
  --stage static \
  --steps 250000
```

The sweep resumes each corresponding seed, not four copies of seed 1. To run
only seed 1 initially, append `--seeds 1 --devices cuda:0` within your allocation.

## Validation and checkpoints

Initial validation captures the resumed baseline before any updates. Every
25,000 transitions, both guarded and unshielded validation run on the same 30
scenario seeds. The two runs diverge after actions differ, as expected.

- `validation.jsonl`: guarded results.
- `validation_unshielded.jsonl`: unshielded results.
- `best_guarded.zip`: highest guarded-success checkpoint with safety/intervention
  tie-breakers; this preserves the previous guarded selection strategy.
- `best.zip`: highest minimum success across both modes, then lower combined
  collision/cliff rates, then greater combined success, then fewer unshielded
  unsafe commands. This can trade guarded performance for unshielded performance;
  inspect both reports before selecting a candidate.
- `final.zip`: last training state, not necessarily the strongest.

Evaluate candidates on validation while tuning, including an empty-stage
regression check. Use a held-out test set only after selecting the configuration.
Example guarded validation (add `--unshielded` and change the output name for
the ablation):

```bash
python -m kenny_rl.evaluate \
  --model runs/server-static-v3-avoidance/seed_1/best.zip \
  --stage static --split validation --seed 10100 --episodes 100 \
  --output artifacts/static-v3-avoidance/seed_1-validation.json
```

Reports include `unsafe_command_fraction` in both modes. Intervention rates
count only enforced stops; an unshielded run can therefore have zero
interventions and many unsafe commands. Reward magnitudes across the old/new
profiles are not directly comparable. Compare success, contacts, timeouts,
route availability, and intervention causes instead.

## Expert distillation after PPO plateaus

On static validation seeds 12000–12099, the non-privileged route follower
completed 100/100 episodes unshielded with zero collisions. It uses estimated
route lookahead and goal distance already represented in the policy input. The
continued PPO checkpoint produced exactly the same guarded and unshielded
results as its source `best.zip`, showing that checkpoint selection retained
the initial model rather than finding an improvement.

`configs/server_static_distill.json` therefore performs behavioral cloning on
200 successful, unshielded route-follower episodes when resuming, saves the
post-cloning actor as `distilled.zip`, evaluates it before PPO updates, and then
runs 100,000 conservative PPO steps. The source run remains untouched. This is
policy distillation from a conventional controller in simulation; it does not
prove real-world performance or make the controller itself a safety authority.

```bash
python scripts/server_sweep.py \
  --config configs/server_static_distill.json \
  --output runs/server-static-v3-distilled \
  --resume-from runs/server-static-v3-avoidance-continued \
  --resume-checkpoint best.zip \
  --stage static --steps 100000 \
  --seeds 2 3 --devices cuda:0 cuda:1
```

Behavior cloning runs before PPO progress tables and can be quiet while it
collects demonstrations. Each run writes `distilled.zip`, `best_guarded.zip`,
and dual validation logs. Evaluate `best.zip` and `distilled.zip`; PPO may fail
to improve the distilled actor, and `final.zip` is not automatically preferred.
