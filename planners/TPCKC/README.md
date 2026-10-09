# Trajectory planning with cumulative key constraints

Zijun Guo, Yuanxin Wang, Huilong Yu and Junqiang Xi, *Fast Optimization-Based Trajectory Planning With Cumulative Key Constraints for Automated Parking in Unstructured Environments*, IEEE Transactions on Vehicular Technology 74(8), 2025, pp. 11820–11830. [DOI](https://doi.org/10.1109/TVT.2025.3555954) · [Authors' visualization and accepted-manuscript links](https://github.com/Easy121/visTPCKC).

`RunPlanner('TPCKC',caseId)` returns a trajectory. This independent MATLAB/AMPL implementation follows equations (6), (12), (14)–(22) and Algorithms 1–2. It uses the common vehicle dimensions and limits; the paper's 0.75 rad steering bound is replaced by the frozen benchmark's 0.7 rad bound. The original solver combination is CasADi/Ipopt/MA97 plus CVXPY/ECOS for dual initialization. Here AMPL/Ipopt/MA97 solves the same trajectory NLP, and the separable, pose-fixed dual SOCP is solved analytically with a geometric certificate.

## Initialization and native optimal control problem

The common Hybrid A* routine supplies an exact circular-arc path. Its disclosed search uses 0.2 m XY and 5 degree heading cells, 0.04 m collision sampling, 0.12 m physical-body clearance, three steering samples, and a 180 s / 100,000-expansion budget. These frontend numerical settings are implementation choices; they are not claimed to be the authors' unavailable HA code.

Each gear run receives an exact rest-to-rest triangular/trapezoidal speed profile. The sampled path's mileage is inverted through this profile to obtain time knots; cubic spline interpolation of XY and continuous body heading is performed separately within each gear run. The initial time is rounded upward to a multiple of the paper's `hr=0.04 s`, with a corresponding uniform slowdown of this initial guess. Speed and acceleration are initialized from the profile; steering and steering rate are **zero**, following Section IV-A. The fixed number of intervals is `N=ceil(PMPtime/hr)`, not a universal 200-node replacement.

The native model is the paper's **implicit Euler** bicycle model: the XY-heading increments use the next state, while acceleration and steering rate are held on the current interval. The objective uses **explicit Euler** quadrature of `100*T + integral(5*(a^2+v^2*omega^2)+10*phi^2)`. Endpoint XY-heading, zero speed and zero steering are hard constraints. Acceleration and steering rate are bounded but not fixed at the endpoints. The goal heading representative is chosen modulo 2*pi to agree with the HA winding; the paper's heading limits are then the smaller endpoint heading minus pi and the larger endpoint heading plus pi.

XY trust boxes are centered on the current warm start. The paper scales XY by the trust radius, heading by pi, speed/steering by their bounds, and inputs by their respective bounds; these scalings are explicitly present in `model.mod`. Physical outputs are converted back to benchmark units.

## Vertex constraints and the cumulative loop

There are two constraint types: a vehicle corner outside an obstacle polytope, and an obstacle vertex outside the rotated vehicle rectangle. Each uses a nonnegative dual vector with normal norm at most one and separation at least `1e-6 m`. These are the literal point/polytope constraints, not the full OBCA separating-set formulation.

The first NLP has no collision constraints and a 1 m trust radius. If its result has vertex intrusions, the corresponding keys are collected and propagated to `round(0.5/0.04)=13` earlier/later node indices, excluding both task endpoints. Newly introduced constraints are added permanently. The first intermediate NLP is warm-started from HA, while later intermediate NLPs use the preceding optimized states and controls. Their trust radius is 1.5 m. Once a native result is vertex-clear, another NLP is solved as the required final trial with a 1 m trust radius. If the trial collides, accumulation continues; a failed intermediate or final solve remains a native failure.

Each new key's dual is initialized **once**, using equation (22) at that iteration's warm-start pose. Previous SOCP values are retained and reused as the warm-start dual list, rather than silently replacing them with the last NLP's optimized multipliers. Since each point/polytope SOCP is independent, `PointDual` obtains its exact optimum from the closest polygon point and expresses the supporting unit normal with nonnegative active-face coefficients. A point inside the polygon has optimum zero and uses a zero dual vector. Every initialization checks nonnegativity, the normal bound and primal/dual distance equality. This is a solver replacement for the same SOCP, not a different warm-start objective.

**Vertex exclusion misses polygon crossings without vertex intrusion.** Section II explicitly acknowledges this enlarged feasible region. The implementation preserves it, including the native stopping test. No full-rectangle filter is inserted to turn this method into another collision formulation. The common evaluator independently measures full-body execution collisions, and release checks also measure native physical collisions separately. No additional obstacle inflation is applied in the NLP; the frozen benchmark geometry is used directly.

## Numerical settings and output

| Parameter | Setting |
|---|---|
| Objective weights `w1,w2,w3` | 100, 5, 10 (Table I) |
| Initial resampling / propagation time | 0.04 s / 0.5 s (Table I) |
| First / intermediate / final trust radius | 1 / 1.5 / 1 m (Table I) |
| Ipopt tolerance / acceptable tolerance | 1e-8 / 1e-8 |
| Native feasibility check | At most 1e-6 residual |
| Per-NLP limits | 2,000 iterations / 180 CPU seconds |
| Resource guards | 20 outer iterations, 2,000 intervals, time in [0.1,600] s |

The last three rows are disclosed implementation/resource choices. A native success requires Ipopt's successful exit flag, independent checks of the implicit equations and all active constraints, and a vertex-clear final trial. Results retain failure flags and diagnostics. Repeated detected keys with no new constraint terminate as a reported failure instead of an unbounded loop.

The trajectory includes all optimized timestamps and physical states. Interval controls are exported at their left nodes, with the last control repeated at the terminal node to complete the common schema. This convention and the implicit-Euler discretization do not imply continuous-time dynamic or collision feasibility between nodes. The common executor uses its documented interpolation and high-accuracy model independently.

AMPL/Ipopt with MA97 and MATLAB Navigation Toolbox are required. The analytic dual solver itself uses base MATLAB; the independent unit test uses Optimization Toolbox to compare against a primal quadratic projection. All online HA, resampling, dual initialization, NLP iterations and output conversion are included in `RunPlanner` wall time. This differs from the article's sum of Ipopt-only CPU times and should not be directly compared with them. Temporary solver files and logs are kept outside the repository.

`TestTPCKC` checks dual optimality against an independent quadratic-program projection, verifies temporal propagation, and retains a crossing-without-vertex-intrusion counterexample. Release verification recomputes objective terms, native model/constraint residuals, boundary conditions, cumulative-key behavior and physical collision measurements without repairing the planner's output.
