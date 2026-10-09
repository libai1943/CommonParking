# Geometric subdivision with Reeds–Shepp shortening

Jean-Paul Laumond, Paul E. Jacobs, Michel Taïx and Richard M. Murray, *A Motion Planner for Nonholonomic Mobile Robots*, IEEE Transactions on Robotics and Automation 10(5), 577–593, October 1994, DOI [10.1109/70.326564](https://doi.org/10.1109/70.326564). An author-hosted copy is indexed by [CaltechAUTHORS](https://authors.library.caltech.edu/records/606zf-tfc37).

This implementation follows the three stages of Section III: a collision-free geometric path, recursive approximation by shortest nonholonomic curves, and random collision-free shortening. It does not use Hybrid A* as an initializer. It returns a **path**, with bounded curvature but potentially discontinuous steering and forward/reverse cusps.

## The three stages

1. **Holonomic geometric planning.** The rigid vehicle may translate and rotate freely in SE(2), temporarily ignoring its rolling constraint. Section III-A explicitly permits any geometric planner returning a path wholly in free space. Here a bidirectional geometric RRT-Connect supplies that path, with shortest-angle linear pose interpolation. A direct geometric connection is tried first. The paper's original polygon decomposition and disk Voronoi implementations are not reproduced.
2. **Recursive subdivision.** First attempt the shortest obstacle-free Reeds–Shepp curve between the endpoints of the whole geometric path. If it collides, bisect the geometric path parameter and recurse on the two halves. Subdivision continues until every shortest curve is collision-free or an explicit resource bound is reached. The geometric path is parameterized by accumulated XY distance plus body-radius-weighted orientation change, a disclosed implementation choice. The Reeds–Shepp steering function uses MATLAB `reedsSheppConnection`, with equal forward and reverse length costs and the common minimum turning radius.
3. **Random shortening.** Draw two configurations uniformly by mileage on the current feasible path. Replace their intervening section with the shortest Reeds–Shepp connection only when it is collision-free and reduces length. Stop after repeated unsuccessful trials or the disclosed resource bound. This is the iterative relaxation of Section III-C; it is not a global shortest-path certificate. The initial/final lengths and accepted shortcut count are reported.

The common rear-axle reference, rectangular vehicle footprint and minimum turning radius apply throughout. No parking-slot classification, manually chosen subgoal or additional scene annotation is required.

## Continuous geometric collision checks

Section III-B2 checks intersections of circular vertex traces with obstacle edges, and obstacle vertices with moving robot edges. `ArcsFree` implements both tests. A robot undergoing a constant-curvature segment rotates rigidly about its instantaneous centre. The second contact test is evaluated by rotating the obstacle vertex in the opposite direction against the initial robot edge. Circle/segment intersections and signed swept angles identify contacts over the entire primitive. Straight motion uses the exact convex hull of its start and end footprints. Contact is a collision; conservative floating-point tolerances apply. This is analytic primitive checking, rather than accepting an arc merely because finitely many sampled footprints happen to be clear.

Geometric SE(2) interpolation in the first stage combines translation and rotation. It is checked by adaptive interval subdivision. At each midpoint, the separating-axis gap supplies a conservative distance lower bound. The maximum displacement of any body point is bounded by `XY_distance + body_radius*abs(angle_change)`. An interval is accepted only if its midpoint gap exceeds the required clearance plus half that displacement bound. Uncertified intervals are split; reaching the depth limit is failure. Thus a failure to certify an edge is not a declaration that it is physically obstructed.

## Configuration and reproducibility

All following values are implementation choices; they are not attributed to numerical settings absent from the 1994 paper. The geometric planner uses a 1 m bound on the weighted pose step, goal bias 0.1, a sampling box covering the task and obstacles plus 3 m, 0.12 m clearance, up to 20,000 iterations and 40 s. Its collision certificate allows eighteen subdivision levels. Nonholonomic subdivision allows depth 22, 12,000 connection calls and 90 s. Shortening allows 1,500 attempts, 150 consecutive failures, 30 s and a minimum length improvement of 1e-5 m. The deterministic random seed is `94000+caseId`; the caller's RNG state is restored.

These finite resource bounds and the randomized geometric planner mean that this software does **not** inherit the theoretical completeness guarantee of an unbounded exact geometric planner. It is a concrete implementation of the paper's construction, not a repetition of the authors' historical timings or an assertion of globally optimal results. Timing includes all three online stages.

`RunPlanner('LaumondRS',caseId)` returns the standard path structure, sampled uniformly within each gear run with exact cusp indices and steering values. The optional exact circular-primitive geometry is also exported, so the common evaluator can evaluate the same path without replacing its arcs by chords. The evaluator supplies its common longitudinal time law and then performs the same execution optimization used for all methods.

`TestLaumondGeometry` checks three hundred random straight/circular sweeps against a dense independent rectangle test, time-reversal invariance, a thin obstacle between two clear endpoints and a tiny obstacle intersecting an outside front-corner arc. Release validation additionally verifies the geometric route, every final primitive, endpoint poses, curvature limits and non-increasing shortening length.

