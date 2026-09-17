# Static-stage recovery from repeated shield stops

The recovery configuration uses `route_clearance_weight=4.0`. A* penalizes the
first two grid rings outside inflated obstacles, encouraging more room for
turning and braking. These cells remain traversable, so a narrow route is not
removed solely by this preference. Both goal routing and frontier routing use
the cost. Only the planner's known/observed map is used.

The old default remains weight zero for controlled comparisons. Safety thresholds,
sensor dropout, motor disturbances and the 1,200-step episode limit are unchanged.
This is not a guarantee that an old policy will follow the new routes correctly.
Fine-tuning and per-seed validation are required.

## Resume

Sync the updated source to the server before launching a new job. Use a new output
directory and resume each seed's best checkpoint:

```bash
python scripts/server_sweep.py \
  --config configs/server_static_recovery.json \
  --output runs/server-static-v3-recovery \
  --resume-from runs/server-static-v3-continued \
  --resume-checkpoint best.zip \
  --stage static \
  --steps 250000
```

If your most recent completed sweep has a different directory, change only
`--resume-from`. The v3 observation/action and shield contract is unchanged.
Validation runs every 25,000 transitions with 30 episodes rather than the old
100,000/10 schedule. This costs extra evaluation time but makes regressions
easier to see. Resume training inside the server's resource allocation and a
persistent terminal/job so an SSH disconnect does not terminate it.

```bash
tail -n 15 -f runs/server-static-v3-recovery/seed_{0,1,2,3}.log
```

## Diagnose and compare

Evaluation reports now contain `intervention_reason_fractions` and
`no_route_fraction`. Per-episode records contain the reason counts. A stopped
step may have multiple reasons, so their fractions need not sum to the total
intervention fraction. Reasons include obstacle proximity, invalid LiDAR/depth/
floor/downward readings, downward/floor hazards, and localization uncertainty.
No-route pauses are counted separately from safety interventions.

Evaluate the same frozen checkpoint on the same validation scenarios twice,
first with weight 0 and then 4 (change the output filename too):

```bash
python -m kenny_rl.evaluate \
  --model runs/server-static-v3-continued/seed_0/best.zip \
  --stage static --split validation --seed 10100 --episodes 100 \
  --route-clearance-weight 4 \
  --output artifacts/static-clearance/seed_0-weight4.json
```

Compare success, contacts, timeout, intervention reasons and mean steps.
Wider routes can take longer even when they prevent deadlocks. Random sensor
dropout remains a source of legitimate stops. Do not lower guard thresholds
merely to improve these metrics. Reserve test scenarios for the chosen policy.

The locally available checkpoint is an empty-trained v3 policy, not the user's
continued-static server weights. The initial paired 10-episode validation check
improved from 7/10 to 8/10 success and 57.5% to 45.3% interventions; both runs
had zero collisions. This small result supports testing the change, not a claim
that the HPC policies now achieve a particular success rate.

On 20 additional validation episodes (seeds 10100–10119), success stayed 11/20
for both weights, with zero contacts. Interventions fell from 26.7% to 21.6%
and obstacle-related stops from 11.5% to 6.0%, but no-route pauses increased
from 38.9% to 42.5% and mean episode length from 772.8 to 791.2 steps. Across
both samples, successes were 18/30 versus 19/30. These mixed results show fewer
guard stops, not reliable navigation or a statistically established success gain.
Use validation during fine-tuning to check whether the policy adapts to the
changed routes. The automated suite passes 59 tests; a 64-transition resume
smoke test checks training compatibility only, not learning quality.
