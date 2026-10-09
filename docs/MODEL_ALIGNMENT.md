# Vehicle-model alignment

This is a source-level inventory of the numerical planners and their benchmark adapters. It is not a certificate that every implementation is fully aligned or that native success implies executable motion. Per-method result tables describe the code actually shipped. The frozen scenes and the common evaluator are unchanged.

## Common physical contract

The position is the rear-axle midpoint. Wheelbase is 2.8 m; front/rear overhangs are 0.96/0.929 m; width is 1.942 m. The common bicycle equations are

```
xdot = v*cos(theta)
ydot = v*sin(theta)
thetadot = v*tan(phi)/2.8
vdot = a
phidot = omega
```

The prescribed limits are |v| <= 2.5 m/s, |a| <= 1 m/s², |phi| <= 0.7 rad and |omega| <= 0.5 rad/s. Numerical trajectory adapters should use these quantities consistently in constraints and exported fields, with exact task poses and zero endpoint speed/steering. Heading representatives may differ by 2*pi, while interpolation and Euclidean optimization variables need a continuous lift. `BenchmarkConfig.m` and the loaded case are the parameter source.

Forward Euler, backward Euler, Runge–Kutta and collocation are different transcriptions of a physical model; using one does not by itself mean a different vehicle. A path-only optimizer may instead parameterize the same rear-axle geometry by arc length. Its output must not be mislabeled as a time trajectory. A flatness representation is also admissible, but its derivative-to-state conversion must obey the chain rule.

Where a physical safety margin is introduced for this benchmark, the target is 0.01 m. An area residual, dual-normal lower bound, penalty coefficient, or dimensionless pseudo-distance is not a physical distance margin. Search initialization margins and optimization margins are listed separately in each method's source.

## Source inventory

| Implementation | Current model and adapter status |
|---|---|
| [STC](../planners/STC/README.md) | Common five-state rear bicycle, explicit Euler, exact rest endpoints and common limits. The documented covering-disc sequence is a separate body approximation. |
| [LIOM](../planners/LIOM/README.md) | Common five-state equations enter the penalty formulation; three discs, common limits and disclosed convergence tests. Retained at the accepted configuration. |
| [H_OBCA](../planners/H_OBCA/README.md) | Five-state rear bicycle with RK2, bounded steering-rate input, zero endpoint steering and 0.01 m separation. |
| [TDR_OBCA](../planners/TDR_OBCA/README.md) | Five-state rear bicycle with explicit Euler, native acceleration/steering-rate inputs, hard whole-task terminal state and 0.01 m separation. Temporal QP and distance-dual mechanism retained. |
| [HyperplaneOCP](../planners/HyperplaneOCP/README.md) | Common five-state RK4 model and rest endpoints. `normalMinimum` limits a separating-plane normal, not obstacle inflation. |
| [TriangleArea](../planners/TriangleArea/README.md) | Supplied Triangle21 formulation adapted to the common rear-axle `tan` model and explicit Euler. The triangle-area tolerance is in m², not a distance buffer. |
| [BOMP](../planners/BOMP/README.md) | Rear-reference turning equation, but the shipped pseudospectral formulation does not bound acceleration through a velocity-state equation; final steering is free. A complete five-state adapter remains unresolved. |
| [SLiFS](../planners/SLiFS/README.md) | Common bicycle equations and limits in successive linearization; three covering discs and 0.01 m clearance. |
| [DL_IAPS_PJSO](../planners/DL_IAPS_PJSO/README.md) | Common curvature/speed/acceleration limits in separate path and speed QPs. The temporal QP does not constrain the exported steering rate or repair cusp steering jumps. This remains an alignment gap. |
| [Eta3](../planners/Eta3/README.md) | Path-only rear-axle flatness representation, with common steering-derived curvature bound; no native time trajectory is claimed. |
| [TEB](../planners/TEB/README.md) | Pose/time band with soft kinematic penalties and a trajectory export adapter. Its approximate signed-chord speed and averaged yaw-to-steering conversion are not an exact five-state trajectory; the shipped 0.1 m obstacle margin also awaits an effective validated adapter. |
| [SE2_NMPC](../planners/SE2_NMPC/README.md) | Rear-axle pose equations with interval-constant speed/steering; the shipped 0.2 m margin and interval-to-node control mapping remain alignment gaps. The common-state adapter is under numerical validation, not certified here. |
| [VPF](../planners/VPF/README.md) | Four-state rear bicycle with interval-constant steering. Continuous steering adaptation also requires changing the circular-arc protection-frame calculation; changing only the ODE would invalidate that calculation. |
| [RITP](../planners/RITP/README.md) | The shipped polynomial parameter is based on reference-path length. Its printed `v=gear*sdot` mapping omits the optimized tangent magnitude, and x-only alignment does not guarantee endpoint heading. Steering/rate feasibility also requires attention. |
| [AnytimePSRO](../planners/AnytimePSRO/README.md) | Uniform 200-node, five-state explicit-Euler model; exact rest endpoints and common physical limits. |
| [TPCKC](../planners/TPCKC/README.md) | Common five-state backward-Euler model, scaled variables and exact rest endpoints. Its spatial trust region is not obstacle inflation. |
| [DFTPAV_Path](../planners/DFTPAV_Path/README.md) | Rear-reference MINCO/flatness path export with common curvature and native speed/acceleration penalty parameters. The native time law is deliberately not submitted as a feasible trajectory. |
| [IndirectOCP](../planners/IndirectOCP/README.md) | Common five-state bicycle in the canonical state/costate equations, but steering endpoints are free. Fixing them requires the matching transversality conditions and a converged boundary-value solve; this remains unresolved. |
| [PointPotentialOCP](../planners/PointPotentialOCP/README.md) | Common five nodal quantities with Hermite–Simpson pose equations, linear speed/steering, exact rate bounds and zero speed/steering at both endpoints. Point-potential obstacle constraints and SQP remain unchanged. |

[LatticeOCP](../planners/LatticeOCP/README.md), classified with search methods, uses the rear-axle `tan` model in arc length and exports a path. Its spatial steering derivatives must not be interpreted directly as time-domain steering rate. HA+CG also exports a path. Analytic constructions are not required to become general-purpose nonlinear trajectory optimizers.

## How to interpret the gaps

An alignment gap is a reason to inspect an implementation, not evidence that the source method cannot solve parking. A changed model needs a fresh solve and unchanged common evaluation. A successful optimizer flag is insufficient: exact native boundary constraints, native dynamics/limits, the standard output mapping and executed collision/terminal checks are distinct checks. Solver budgets and unsuccessful finite runs must remain visible. Unverified trial code is not included as a replacement planner.
