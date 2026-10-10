# Point-potential optimal-control parking

K. Kondak and G. Hommel, *Computation of Time Optimal Movements for Autonomous Parking of Non-Holonomic Mobile Platforms*, ICRA 2001, pp. 2698–2703, DOI [10.1109/ROBOT.2001.933030](https://doi.org/10.1109/ROBOT.2001.933030). The paper is also listed in the [author's university bibliography](https://pdv.cs.tu-berlin.de/leute/hommel.html).

This is an independent MATLAB realization of the paper's terminal-parking OCP, point-potential constraints and direct-collocation/SQP/refinement loop. **MATLAB `fmincon` SQP replaces SNOPT.** Neither SNOPT nor DIRCOL source is included, and the measured performance is not claimed to reproduce those packages. No learned controller, parking-slot label, Hybrid A* or geometric path initializer is used.

## Mathematical formulation

The states are rear-axle `x,y,theta`; the controls are signed speed `v` and front-wheel angle `phi`. The bicycle equations are `xdot=v*cos(theta)`, `ydot=v*sin(theta)`, `thetadot=v*tan(phi)/L`. Common bounds apply to both controls and their derivatives. The objective is only final time. Both endpoint poses are fixed modulo `2*pi`; both endpoint speeds and front-wheel steering angles are zero. Fixing final steering is the declared benchmark rest-state adaptation to the paper's otherwise free final steering. The five nodal quantities `(x,y,theta,v,phi)` use the common rear-axle model, with bounded `a=vdot` and `omega=phidot`.

Each scene is translated and rotated into its starting-pose frame, then transformed back for output. This changes no physical geometry. The goal heading representative is the shortest wrapped difference from the starting heading. The initial guess therefore uses the same zero starting orientation as the paper's examples without requiring axis-aligned world coordinates.

The polygon-to-sensor adapter samples each obstacle perimeter, retaining every vertex. Each edge receives `ceil(160*edge_length/total_perimeter)` samples, including its first vertex and excluding its last. This produces approximately 160 points across a scene; the exact count is recorded. It is an explicit benchmark adapter for the article's 50–200 detected points. The optimizer uses these points only, not an additional full-polygon collision constraint.

For a point expressed in body coordinates, let `X=abs(x_body)` and `Y=abs(y_body)`. Let `A` be the positive distance from the rear axle to the front or rear boundary, selected by the sign of `x_body`, and `B` be half the body width. Outside or on the rectangle boundary the potential is zero. Inside, the benchmark uses the boundary-continuous potential

```
Pi = min(1-X/A, 1-Y/B);
P = sum(Pi);       P <= 0.
```

`Pmax=1` is a scale choice. Every interior point has positive potential; every boundary or exterior point has zero potential. Thus the point-exclusion feasible set is the same as the printed equation (4). The normalized minimum makes the value continuous at both the branch switch and the rectangle boundary. Piecewise analytic derivatives select the longitudinal branch at an exact tie; absolute-value coordinates at zero use derivative zero. The potential remains nonsmooth, not globally differentiable.

This potential is an explicit benchmark adaptation, **not a literal transcription of equation (4)**. That printed equation selects the longitudinal branch when `A-X >= B-Y`, comparing unnormalized clearances. Its resulting values need not agree at the branch switch or tend to zero at a body boundary, and its branch differs from the paper's prose about the shortest exit direction. The benchmark retains the point-potential exclusion mechanism, while using a continuous normalized field for SQP. It does not claim that the adapted gradient is the physically shortest exit direction. The original SNOPT implementation is unavailable for comparison.

Excluding a finite set of obstacle points is not a full-body safety certificate. Collisions between sampled points and complete containment inside a large obstacle can be missed. The independent common evaluator always uses the complete polygon obstacles and ego rectangle.

## Discretization and finite numerical settings

The article specifies direct collocation and checking/refining the original problem, but does not provide its mesh or tolerances. The choices here are therefore explicit: cubic Hermite poses, piecewise-linear `v,phi`, and Hermite–Simpson dynamics on a uniform normalized-time mesh. Variables are all five node quantities and final time. Rate bounds are the exact differences of piecewise-linear controls divided by interval duration. Point potentials are imposed at nodes. Analytic objective, dynamics, control-rate and potential Jacobians are supplied to SQP.

The initialization follows the paper's alternative of solving the same OCP without obstacles. Its first guess has the stated L-shaped XY path (first to the target X, then to target Y), zero other variables except required endpoint values, and an unspecified-by-paper duration chosen here as 10 s. If that solve fails, one deterministic retry adds `0.05*sin(2*pi*s)` rad to heading, `0.25*sin(2*pi*s)` m/s to speed, and `0.05*sin(pi*s)` rad to steering, restoring fixed endpoint values. This small perturbation is a disclosed choice under the paper's suggestion to retry with a changed initial solution; it breaks the zero-control linearization's degeneracy. No failed obstacle-free solution is accepted as a valid initializer.

After initialization the full point-constrained OCP is solved. Meshes are 51, 101 and at most 201 nodes. After each successful SQP solve, every interval is checked at eight or more equal subintervals, with checking spacing at most 0.01 s. The maximum component of `Hermite_pose_derivative - bicycle_rhs` must be at most `1e-4`; sampled point potential and bound violation must be at most `1e-8`. If a check fails, the mesh is doubled and the previous Hermite/linear representation supplies the new guess. Exhausting the mesh limit reports failure. These are finite sampled checks, not a continuous collision proof.

Each SQP call has 600 iterations, 10,000 function evaluations, a 60 s limit checked between iterations, constraint tolerance `1e-8`, optimality tolerance `1e-7`, and step tolerance `1e-10`. An in-flight QP can exceed the time limit. Final time is guarded to `[0.05,600]` s. Success requires a positive solver exit flag, direct constraint and bound residual at most `1e-8`, and the final mesh check. Failed-stage iterates are retained with `status.success=false`; no other planner supplies a fallback. Failures under this backend and finite configuration do not establish infeasibility of a scene or failure of the original SNOPT implementation.

## Standard output and verification

```matlab
result = RunPlanner('PointPotentialOCP',3);
evaluation = EvaluateResult(result,LoadCase(3));
```

The result is a trajectory with timestamps, XY, continuous heading, signed speed, steering, acceleration and steering rate. Output includes a uniform grid with spacing at most 0.02 s, native knots and every exact zero crossing of piecewise-linear speed. Timestamps within 32 floating-point ulps of the horizon scale are merged with priority given to task endpoints, then cusps, then native knots; numerically coincident cusp records map to the retained timestamp. `a` and `omega` are the outgoing interval slopes; at the final time they use the last interval. All original native states, timestamps, solver statuses and mesh checks remain in diagnostics. The common evaluator uses the standard linearly interpolated submitted samples, then performs its own execution optimization and integration.

`TestPointPotentialOCP` checks model and potential derivatives, the full collocation Jacobian, rigid-frame invariance, boundary and branch continuity, a nontrivial curved rest-to-rest solve, Hermite interpolation/derivatives and exact off-grid cusps. Release verification recomputes native equations and mesh checks and independently integrates the linear controls at steps no larger than 1 ms. Full-body overlaps of native nodes and that replay are diagnostic and do not replace the common measured execution metrics.

The planner needs MATLAB Optimization Toolbox and has no Navigation Toolbox or AMPL dependency. The common evaluator has its separately documented runtime requirements. Complete planner wall time includes setup, loading, both initialization attempts when needed, all mesh solves/checks and output conversion. One numerical-library thread is used in the recorded runs; concurrent development jobs mean the timing table is not a controlled hardware comparison.
