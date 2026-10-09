# TriangleArea — Triangle21 formulation for CommonParking

Bai Li and Zhijiang Shao, *A unified motion planning method for parking an autonomous vehicle in the presence of irregularly placed obstacles*, Knowledge-Based Systems 86 (2015), 11–20, DOI [10.1016/j.knosys.2015.04.016](https://doi.org/10.1016/j.knosys.2015.04.016).

This planner adapts the user-supplied, previously tested **Triangle21** implementation. The vehicle equations, objective, Euler transcription, explicit corner variables, endpoint conditions and two-sided triangle-area constraints follow that implementation. CommonParking supplies its own vehicle limits and tasks. This is not a reproduction of the paper's original experimental settings.

## Technical model

The state is `[x,y,theta,v,phi]`, with XY at the rear-axle midpoint. Controls are acceleration and steering rate:

```
xdot = v*cos(theta), ydot = v*sin(theta),
thetadot = v*tan(phi)/wheelbase, vdot = a, phidot = omega.
```

There are 200 uniformly timed states and 200 control values. First-order explicit Euler uses controls 1 through 199; final acceleration and steering rate are fixed to zero. Initial and final pose are equalities; both ends have zero speed and steering. Final time is minimized with bounds `[0.1,100]` s. The goal heading uses the continuous branch reached by the initial path, equivalent to the prescribed heading modulo 2*pi. Optimization headings are not wrapped to a principal interval.

The eight corner coordinates `AX,AY,...,DX,DY` are independent variables linked to rear-axle pose by eight equalities per node, as in Triangle21. Each vehicle vertex must be outside every obstacle and every obstacle vertex outside the vehicle, according to triangle-area sums. The supplied **0.01 m² area excess** is preserved without normalization. It is not a 1 cm distance buffer. Obstacles are not geometrically inflated. The single-polygon indexing is extended to multiple individually closed polygons; areas and vertex cycles remain separate.

Wheelbase is 2.8 m, front/rear overhangs 0.96/0.929 m, and width 1.942 m. Forward and reverse speed limits are 2.5 m/s, acceleration/braking magnitudes 1 m/s², steering magnitude 0.7 rad, and steering rate 0.5 rad/s. These values are read from the common case vehicle, not copied from Triangle21's example, which allowed forward motion only.

## Initialization and numerical settings

The supplied example loads a task-specific `perfect_ig.mat`. Here Hybrid A* generates an initial path for the selected benchmark case. Adjacent primitives with the same gear and curvature are merged; the seed stops at curvature changes and adjusts steering at a bounded rate before continuing. Rest-to-rest longitudinal timing is applied on each arc. Heading is accumulated continuously through branch crossings and gear reversals. This continuous seed is sampled at the 200 planning nodes; it is not claimed to satisfy discrete Euler equations exactly. All eight corner variables are explicitly initialized from its poses.

A single minimum-time NLP is solved. There is no separate projection objective, penalty continuation or alternative collision formulation. Ipopt retains Triangle21's `max_cpu_time=60`, `tol=1e-10`, `bound_push=1e-4`, adaptive barrier strategy and MA97. The iteration cap is 3,000, and an external process bound is 150 wall seconds. Numerical-library threads are fixed to one. A local AMPL/Ipopt installation is required and is not redistributed.

Search uses a 0.2 m / 0.2 rad grid, 0.7 m primitives, three steering samples, heuristic weight 3, reverse penalty 1, gear-change penalty 5, zero clearance inflation, 100,000 expansions and 180 s. Public planning time includes search, initialization, the NLP and output conversion.

## Output and checks

Full-precision states and controls are exported in the standard timestamped trajectory format. The native termination message must report an optimal or acceptable solution with a successful numerical code. Discrete dynamics and corner equalities must have residual below `1e-6`. Like Triangle21's `IsCurSolValid`, the planner then applies a separating-axis test to the complete rectangle at every planning node. An intersecting node makes the result unsuccessful even if Ipopt converged. Raw native convergence remains separately recorded.

The common evaluator performs its own execution and full-body collision checks, with no changes to the shared protocol. Zero native node collisions does not guarantee zero executed collision frames. Euler discretization, between-node motion, and the vertex-only constraint's crossing-edge limitation remain relevant. Failed finite candidates can be plotted but are not scored as successful plans.

`TestTriangleEuler` checks heading winding, principal-angle crossing, gear reversal, endpoint geometry and initialization motion bounds. Published records are independently checked for equations, endpoints, limits, area inequalities and full-body node clearance. The validation report also verifies the model mapping to Triangle21 and the twelve-case initial-heading audit.

See [validation](../../results/TriangleArea/validation.json), [twelve-case data](../../results/TriangleArea/metrics.csv) and the root README table.
