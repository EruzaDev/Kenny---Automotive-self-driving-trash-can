# Progressive mapping and local-policy fine-tuning

`EnvConfig.map_mode="progressive"` starts with no structural wall map. Valid
LiDAR rays reveal free cells; returns mark occupied cells, inflated for the
robot footprint. Occupied observations persist until a subsequent free ray
clears them. Depth and floor hazards augment the planner's obstacle memory.
Invalid rays reveal nothing. Mapping uses estimated acquisition pose only.

Conventional A* routes through observed cells. When no observed route reaches
the supplied goal, a conventional frontier selector chooses a reachable free
cell bordering unknown space. A selected frontier stays active until reached,
no longer a frontier, or blocked. Replanning occurs every second. The PPO actor
continues to consume local route lookahead, sensor features and localization
health; it does not learn mapping or frontier selection.

This is a bounded-grid mapping surrogate for training, **not a real SLAM
implementation**. It assumes known map bounds, an initialized estimated pose,
and goal coordinates in the same frame. Marker pose corrections remain
simulated; there is no camera detection, scan matching, loop closure or map
realignment after a localization correction. Sparse rays, drift and dynamic
obstacles can leave holes or stale cells. Frontier selection is a heuristic,
not a guarantee of complete exploration. LiDAR free space does not prove safe
floor support; the independent floor/depth guard remains necessary.

## Fine-tune existing v3 checkpoints

The observation/action contract is unchanged, so v3 policies can resume. Map
mode and episode budget are saved in `config.json`, and source hashes record
the changed mapping and reward implementation. Older test scores do not
qualify a policy for progressive mapping. Start from your successful disturbed
empty sweep, initially retaining empty geometry:

```bash
python scripts/server_sweep.py \
  --config configs/server_progressive.json \
  --output runs/server-progressive-empty-v3 \
  --resume-from runs/server-empty-v3 \
  --stage empty \
  --steps 250000
```

Evaluate each seed before advancing. For seed 0:

```bash
python -m kenny_rl.evaluate \
  --model runs/server-progressive-empty-v3/seed_0/best.zip \
  --stage empty --split test --episodes 500 \
  --output artifacts/progressive-empty-v3/seed_0.json
```

The saved progressive mode is automatically reused. Reports include map mode,
mean steps, and per-episode mapped fraction and navigation mode. Compare success,
contacts, timeouts, interventions and steps, not reward alone. After successful
evaluation across seeds, resume into `--stage static` with the same config and
a new output directory, then evaluate that stage before increasing difficulty.

For a baseline on an existing checkpoint, evaluation also accepts
`--map-mode progressive`; this does not alter the checkpoint or its saved config.

## GUI

On a desktop machine with the checkpoint directory copied locally:

```bash
python -m kenny_rl.demo \
  --model runs/server-progressive-empty-v3/seed_0/best.zip \
  --stage empty --map-mode progressive \
  --start 1 1 --goal 11 11 --steps 2400 --seed 42
```

Gray shading shows unseen space; the title distinguishes exploration from goal
following. Visible obstacle geometry underneath is debug ground truth only.

## Final approach

Routes now end at the exact requested goal rather than a grid-cell center.
The previous recurring reward for low motion near the goal is removed: lingering
no longer earns a positive reward. Within 0.40 m, shaping favors a distance-based
braking speed that reaches zero at 0.20 m, inside the 0.25 m arrival tolerance.
This is a training objective, not a forced minimum speed; avoidance and the guard
can always stop motion. Existing policies need fine-tuning to learn the changed
reward. The demonstration follower uses the same braking profile. Arrival still
requires low linear and angular speed plus true and estimated goal proximity.

For hardware, connect a tested SLAM/localization stack and exploration planner
to the same observation interface; simulated mapping success is not evidence
that those hardware integrations already exist.
