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
