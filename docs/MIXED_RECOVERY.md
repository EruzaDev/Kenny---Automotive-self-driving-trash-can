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

## Forward LiDAR validity coverage

Mixed validation after planner fine-tuning still attributed about 19.2% of all
steps to `lidar_invalid`.  The old guard required every forward LiDAR ray to be
valid.  With 15 forward rays and independent 1.5% simulated dropout, the chance
of at least one invalid ray is about 20%, so ordinary isolated dropout was being
treated like a complete sensor failure.

The guard now accepts isolated or two-ray gaps when neighbouring rays retain
coverage.  It stops for three adjacent invalid forward rays or less than 80%
aggregate forward coverage.  Complete outages therefore still stop motion.
Obstacle distance checks, depth/floor/downward validity checks, localization
health, and the physical shield remain unchanged.  This is a simulator rule
that must later be calibrated against the real LiDAR's angular sampling,
correlated failures, stale scans, and diagnostic status; it is not permission
to treat unknown space as free.

On the same 100 guarded mixed validation seeds used for the debug follower
check above, success remained 88/100 with zero contacts.  Total interventions
fell from 39.1% to 27.0%, and `lidar_invalid` stops fell to 1.38% of steps.
No-route time was 6.5%.  The unchanged success count shows that removing false
stops is necessary but not by itself evidence that the frozen PPO policy or the
planner has met the mixed-stage release target.
