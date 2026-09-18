# Mixed-stage temporary-obstacle recovery

Paired mixed validation exposed planner blockage rather than only a PPO
weakness.  In the reported 200-episode evaluations, 14/18 and 15/19
unshielded timeouts spent more than half of the episode without a route.  Those
timeouts averaged 774 and 805 no-route steps respectively.

Sensor endpoints are points inside grid cells.  Temporary obstacle memory now
inflates an endpoint by the robot radius plus half of the grid cell diagonal.
The former radius-plus-one-full-cell rule also blocked diagonal neighbours at
the default 0.25 m resolution.  Multiple endpoints could therefore seal a
route that the generated world's exact-geometry connectivity check had shown
to be traversable.  Known structural walls retain their existing inflation,
and visible unknown obstacles continue to refresh their finite memory.

This change does not relax the collision shield, command limits, or the rule
that the robot stops when no route exists.  A deliberately less conservative
endpoint-only alternative was rejected because it produced a collision in the
initial local paired-seed route-follower check.  On validation seeds
14000--14099, the old and selected grid-consistent rules gave respectively
90/100 and 92/100 successes, 14.9% and 8.2% no-route steps, and one and two
contacts with the shield disabled.  The selected rule therefore reduces false
blockage but is not evidence for unshielded deployment.  With the shield
enabled, the selected rule's debug follower completed 88/100 with zero contacts,
12 timeouts, 5.7% no-route steps, and 39.1% interventions.  These are regression
checks of a hand-written follower, not claims about a trained PPO checkpoint.

Existing checkpoints remain compatible because observation fields, action
scaling, and the serialized model contract are unchanged.  Re-evaluate the
frozen mixed checkpoint with the updated source before fine-tuning.  Use the
same validation seeds and keep guarded and unshielded reports separate.  Do
not select or tune on the held-out test split.

## Rejected forward LiDAR validity relaxation

Mixed validation after planner fine-tuning still attributed about 19.2% of all
steps to `lidar_invalid`.  The old guard required every forward LiDAR ray to be
valid.  With 15 forward rays and independent 1.5% simulated dropout, the chance
of at least one invalid ray is about 20%, so ordinary isolated dropout was being
treated like a complete sensor failure.

An experiment accepted isolated or two-ray gaps while stopping for three
adjacent invalid rays or less than 80% aggregate coverage.  On a 100-episode
debug-follower check it retained 88/100 success and zero contacts while reducing
interventions from 39.1% to 27.0%.  However, the decisive 200-episode frozen-PPO
validation produced one guarded collision, compared with zero under the
original rule, for only a 0.5 percentage-point success increase.  The
relaxation was therefore rejected and the guard again requires every forward
LiDAR ray to be valid.

Real hardware may eventually use temporal filtering or a driver-level scan
health model, but only after recorded-sensor validation demonstrates that it
does not hide thin or newly appearing obstacles.  Independent per-ray dropout
in this analytical simulator is not sufficient evidence for weakening the
safety rule.

## Blocked goal-cell tolerance

The downloaded fine-tuning run repeatedly failed validation scenario 10025.
The robot travelled about 7 m and then spent 810--890 of 1,200 steps without a
route.  Replay showed that the exact-geometry and structural-map planners still
had valid routes.  Eight temporary cells near an overhang blocked the coarse
grid cell containing the goal, even though the overhang was 0.396 m from the
continuous goal and the robot radius was 0.163 m.

Known-map planning now retains exact-goal routing as its first choice.  Only
when temporary occupancy blocks the goal cell, it may route to the closest
reachable free cell strictly inside the existing 0.25 m arrival radius.  It
does not clear temporary occupancy, route through an occupied cell, expand the
arrival criterion, or bypass a blockage elsewhere.  Progressive mapping keeps
its existing frontier behavior.  The environment still requires both true and
estimated pose to be inside the unchanged arrival radius at low velocity.

## Guarded in-place recovery

The 200-episode goal-tolerance evaluation left 17 timeouts.  Nine retained a
valid route but spent 78--96% of the episode under shield intervention, often
with a frontal obstacle or invalid depth view.  The former guard stopped both
translation and rotation, so a policy requesting a turn could not change its
view or heading.

For frontal LiDAR/depth obstacles, their validity failures, and localization
uncertainty, the guard now clamps linear velocity to zero while preserving the
requested angular velocity.  The simulator collision footprint is the robot's
circumscribed circle, so an in-place turn adds no swept area.  Downward hazard,
downward invalidity, floor hazard, and floor invalidity remain full stops; the
recovery cannot override them.  Shadow guard behavior remains non-actuating in
explicitly unshielded evaluation.

Before implementation, targeted frozen-policy replay recovered six of the nine
route-available shield deadlocks with full requested angular velocity and no
contacts.  This targeted result requires a new full guarded validation; it is
not a release claim.

## Fine planner grid

The guarded turn-recovery evaluation reached 188/200 successes with zero
contacts.  Five of the remaining timeouts lost their route even though a
diagnostic planner built from exact simulator geometry could still reach the
goal.  Rather than weaken footprint inflation, `configs/server_mixed_finegrid.json`
halves the planner cell size from 0.25 m to 0.125 m.  This increases grid cells
by approximately four times in a fixed-size room, so runtime must be measured.
It does not change policy observations or actions.

On the five previously blocked scenarios, frozen-policy replay restored routes
for all five, produced four successes and one long-route timeout, and produced
no contacts.  Evaluate the complete frozen checkpoint before using the profile
for fine-tuning:

```bash
python -m kenny_rl.evaluate \
  --model runs/server-mixed-v3-planner-finetune/seed_3/best.zip \
  --stage mixed --split validation --seed 15000 --episodes 200 \
  --grid-resolution 0.125 --progress-every 20 \
  --output artifacts/mixed-v3-planner-finetune/best-guarded-finegrid.json
```

The report records `grid_resolution` explicitly.  Do not compare it with an
older report without checking that field.  If frozen validation is safe but
requires policy adaptation, resume into a new run with the fine-grid profile;
never edit a completed run's serialized configuration in place.

The policy route lookahead is sampled at fixed physical arc lengths of 0.5 m,
1.0 m, and 2.0 m.  It is interpolated along the route from the robot's closest
route projection.  This preserves the observation meaning across planner grid
resolutions; the previous fixed waypoint offsets `[2, 4, 8]` accidentally
halved the lookahead horizon on the 0.125 m grid.  Re-evaluate the frozen
checkpoint with this correction before fine-tuning it.

## Forward-camera safety envelope

Held-out mixed testing exposed collisions with low bags and body-height
overhangs that do not intersect the 0.22 m LiDAR plane.  The forward Astra is
the only configured sensor that can observe these hazards, and it has no side
coverage.  The shield therefore suppresses translation whenever either the
requested or current angular velocity exceeds 0.55 rad/s, while preserving
the turn command.  This rotate-before-translate behavior lets the camera view
the intended direction before the robot advances.  Depth hits also receive a
0.15 m obstacle buffer instead of the planar scan's 0.05 m buffer.

These controls reduce risk but cannot make an unobserved obstacle observable.
Physical deployment still requires a bumper/emergency stop and conservative
commissioning speeds.  They must not be described as a collision guarantee.

## Dynamic-contact accounting

Ordinary train, validation and test pedestrians now maintain clearance from
the robot instead of walking directly into a stopped platform.  The stress
split deliberately retains non-cooperative pedestrian motion.  Dynamic and
full stages add a worst-case 0.8 m/s pedestrian closing-distance allowance to
the sensor guard.  Evaluation records the collision source, linear/angular
contact speed, whether the shield commanded a stop, and separate person,
stationary-person and robot-motion collision rates.  Total collision rate is
retained for backward-compatible reporting.
