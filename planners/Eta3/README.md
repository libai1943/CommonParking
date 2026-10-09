# Multi-optimization of eta3 splines

Gabriele Lini, Aurelio Piazzi and Luca Consolini, *Multi-optimization of η³-splines for autonomous parking*, IEEE CDC/ECC 2011, pp. 6367–6372, DOI [10.1109/CDC.2011.6161095](https://doi.org/10.1109/CDC.2011.6161095). [Author-hosted paper](https://www.ce.unipr.it/people/piazzi/documents/2011-Lini-et-al-CDC-conf.pdf).

`RunPlanner('Eta3',caseId)` returns a path. It implements the paper's simplified seventh-degree eta3 curve family, geometric endpoint conditions, cusp curvature coupling and weighted nonlinear program, with an explicitly disclosed online initialization adapter. This is not an implementation of an unpublished offline lookup table and is not claimed to be the authors' original software.

## Curve family and optimization variables

One seventh-degree polynomial in each Cartesian coordinate represents each forward or reverse maneuver. For a curve endpoint with geometric tangent T, left normal N, curvature k and mileage derivative dk, the simplified family uses parameter derivatives

```
P' = eta*T,   P'' = eta^2*k*N,   P''' = eta^3*dk*N.
```

Together with the prescribed position, these four endpoint conditions at each end determine the eight polynomial coefficients. `Curve` solves the constant four-by-four remainder system; it is algebraically equivalent to the expanded coefficients in Section III. It does not substitute cubic or quintic splines. The tangent, curvature and curvature derivative follow analytically from the polynomial derivatives.

With h alternating-gear maneuvers there are exactly **8h-4 variables**, as in Table I: two curvature-derivative endpoint values per maneuver, two positive endpoint parameter speeds per maneuver, and a shared `(x,y,theta,kappa)` at each of the h-1 cusps. Physical heading and curvature are shared at a cusp; reverse curves use geometric tangent `theta+pi` and curvature `-kappa`. Geometric curvature derivatives on opposite sides of a stop are independent, as in the paper's h=2 construction. Initial and final steering are zero. Each regular polynomial is internally G3; the shared physical curvature avoids steering jumps at stationary reversals.

The objective is equation (11), with the paper's example weights:

```
0.5*max(abs(curvature)) + 0.2*max(abs(dcurvature/dmileage))
                         + 0.3*total_length.
```

The maximum curvature uses the benchmark vehicle limit. The curvature-derivative bound is the paper's 2.5/m². Every pose uses the complete rectangular vehicle and all polygon obstacles. The generic nonintersection condition (12) is expressed through separating-axis inequalities; the paper's optional maximum-overlap-area equality/penalty conversion is not used. Boundary contact is permitted at zero separation, subject to the NLP tolerance.

## Discretization and numerical solver

The paper's experiments use 100 uniform intervals of polynomial parameter u per maneuver. This implementation keeps that mesh for the peak curvature, peak curvature derivative and collision constraints. Length is integrated by eight-point Gauss quadrature over twelve subintervals. The exact polynomial derivatives are used at all test points. A small positive parameter-speed floor, 1e-5 m per unit u, prevents sampled singularities.

**These are sampled constraints.** Native convergence does not prove the continuous sweep safe or bound the curvature between those nodes. The released independent validation reports dense reference checks, and the common evaluator measures the executed result. Sampling overshoot or replay collisions are not hidden by replacing the curve with a different planner.

MATLAB `fmincon` with SQP solves the stated constrained problem. The article proposes standard local minimization but does not identify its exact solver/settings. The implementation uses at most 300 iterations, 50,000 function evaluations or 180 s, with 1e-6 feasibility/optimality and 1e-9 step tolerances. Both a positive native exit flag and recomputed sampled feasibility are required. A stopped or failed solve remains a failure even if its last iterate looks useful. Numerical-library threads are fixed to one during the planner call.

## Initialization adapter and finite bounds

Section IV assumes a chosen maneuver count, initial gear and parameter estimate, and suggests obtaining them from offline optimization lookup tables. Those tables are not supplied. To provide a reproducible, self-contained online call without collected expert data, ordinary Hybrid A* supplies the maneuver count/order and initial cusp poses. Each eta endpoint speed starts at its maneuver's mileage, endpoint curvature derivatives and shared cusp curvatures start at zero. This is a benchmark initialization adapter, not a claim that the paper specifies Hybrid A*. The subsequent optimizer can move all cusp poses and curvatures; the initial path is not kept fixed. No CG is used, and no search result is returned as a fallback for a failed spline solve.

Finite numerical bounds localize this otherwise unbounded parameter space: at most twelve maneuvers; eta between 0.01 and four times the initial maneuver length plus 2 m (upper bound at least 2 m); cusp positions inside the task/obstacle bounding box expanded by 6 m; cusp heading within pi of its initial continuous angle; cusp curvature and all endpoint curvature derivatives inside their stated physical/geometric bounds. These are disclosed numerical choices. There is no claim of global Pareto optimality or of finding a solution whenever one exists.

After a successful solve, adaptive Gauss quadrature and mileage inversion resample each maneuver at uniform spacing no greater than 0.02 m, preserving every exact cusp and endpoint. The standard result contains mileage, XY, heading, steering, gear intervals and cusp indices. The evaluator uses its generic interpolation between these samples. All search, optimization, quadrature and export work count toward online planner time.

`TestEta3` checks 100 random forward/reverse endpoint interpolations, the analytic curvature derivative against finite differences, a known straight-line NLP, and mileage inversion. Release checks additionally recompute the objective, sampled constraints, native flags, cusp continuity, task endpoints and dense reference footprints.
