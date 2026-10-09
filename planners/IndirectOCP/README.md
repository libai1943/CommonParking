# Indirect optimal-control parking planner

E. Pagot, M. Piccinini, E. Bertolazzi and F. Biral, *Fast Planning and Tracking of Complex Autonomous Parking Maneuvers With Optimal Control and Pseudo-Neural Networks*, IEEE Access 11, 124163–124180, 2023, DOI [10.1109/ACCESS.2023.3330431](https://doi.org/10.1109/ACCESS.2023.3330431), [university record](https://iris.unitn.it/handle/11572/404089).

This folder implements **the planning component, Section III**, including its Hybrid A* followed by optimal-control tracking initialization. The learned pseudo-neural execution controller in Section IV is not used. Every accepted trajectory is measured by the same CommonParking evaluator as the other planners.

## Paper formulation

The five physical states are rear-axle `x,y,theta,v,phi`; the controls are acceleration and physical front-wheel steering rate. They satisfy the common bicycle equations. The paper's steering-wheel transmission ratio is absorbed exactly into the physical steering-rate input and its common bound. Both ends fix `x,y,theta,v=0`. Steering is free at both endpoints, as in equations (6e–f); its costate is therefore zero there. The continuous heading representative of the prescribed goal is selected by the initial HA path; it differs from the given angle only by an integer multiple of `2*pi`.

The final objective is `wT*Tf + integral(obstacle_cost)`. Each rectangle obstacle uses the printed smooth window product in equations (2)–(5). Twelve checking points are placed on the common ego rectangle: four corners, three equally spaced interior points on each long side, and the front/rear edge midpoints. No circle approximation is used. Each supplied obstacle is checked to be an exact rectangle; all twelve benchmark scenes satisfy this condition. Arbitrary non-rectangular input is not silently replaced by a bounding box.

For relative coordinate `s` and side length `L`, the window is

```
Q(s,L) = 0.5*((s+L/2)/sqrt(ht^2+(s+L/2)^2)
             -(s-L/2)/sqrt(ht^2+(s-L/2)^2)).
```

The penalty sums the products of both windows over every checking point and obstacle. The formula has nonzero tails outside the obstacle; the paper's informal zero-outside description is not exact. The method uses a **soft** obstacle objective, and twelve perimeter points can miss overlap between points or containment of a small obstacle. Native convergence does not certify full-body collision avoidance. The common evaluator always checks the full rectangular body independently.

The initial OCP is equation (12): minimum time plus squared tracking errors in `x,y,theta` along the HA reference, subject to the same dynamics, bounds and endpoints. Its state and costate solution initializes the final obstacle OCP. The HA path is assigned the common longitudinal rest-to-rest profile only to define its phase reference and numerical guess. The final OCP is free to change positions, heading, speed, gear changes and duration.

## Open indirect solver and disclosed numerical choices

The article used the authors' proprietary PINS solver. **This is an independently written MATLAB indirect solver, not PINS code or a claim to reproduce PINS runtime.** The continuous Hamiltonian and state/costate boundary-value equations are derived before discretization. The approach follows the authors' public [symbolic-numeric indirect-method description](https://e.bertolazzi.dii.unitn.it/files/PaperMB2003-044.pdf) and [indirect-method review](https://www.jstage.jst.go.jp/article/ieejjia/5/2/5_154/_pdf), DOI [10.1541/ieejjia.5.154](https://doi.org/10.1541/ieejjia.5.154). It does not pass the primal trajectory to a direct NLP optimizer.

The unknowns are five states and five costates at **500 nodes**, as in the article, plus `log(Tf)`. Eliminating the constant duration state and integrating its costate equation gives the equivalent scalar time-transversality condition `wT + mean(H) = 0`. The canonical ODE is discretized by implicit midpoint; endpoint and time equations complete a square sparse nonlinear system.

State speed/steering and both controls use `B(z,b)=-mu*log(1-z^2/b^2)`. The control stationarity equations are eliminated analytically: `u=-b^2*lambda/(mu+sqrt(mu^2+(b*lambda)^2))`. Analytic first and second derivatives form the canonical Jacobian. Sparse LU Newton steps use affine-scaled residual backtracking. Costate initialization is a sparse linear least-squares fit to adjoint, control-stationarity and time conditions; it is only an initial guess, not a primal optimizer.

The paper leaves these settings unspecified: `wT=1`, position and heading tracking weights 10, penalty amplitude 1000, and `ht=0.1 m`. The omitted amplitude scaling of the paper's footnote is represented by the penalty multiplier. The initial state guess has one-third reference speed, 0.96 times reference steering, and three times reference duration. Tracking first converges with `mu=0.1`. An adaptive homotopy replaces the tracking running cost by the complete obstacle cost, ending at weight one; failed trials restore the previous solution. A logarithmic barrier continuation then reduces `mu` to 0.001, adapting its step after failures. No residual tracking term remains in the final objective. Finite regularization is explicitly retained, so this solves an approximation of the hard-bound PMP conditions.

Every required nonlinear solve must reach maximum equation residual below `1e-7`. Newton has 180 iterations, 24 backtracking trials and a 180 s limit per solve. Each continuation has at most 120 attempts and a 180 s outer limit checked between solves; an in-flight solve can exceed the outer deadline. HA has the common 180 s/100,000-expansion budget. Numerical guards keep `0.05<Tf<600 s` and nodal steering away from the tangent singularity. Both continuation targets must be reached to report success; there is no fallback to the initial path or tracking-only solution.

The strict state barriers are evaluated at the **midpoints used by the canonical discretization**. Nodal speed/steering can overshoot their midpoint bounds. These values and the independent integration discrepancy are recorded, never clipped or hidden. First-order convergence does not prove a local minimum, global optimum, continuous-time feasibility or collision avoidance. Failures under this finite solver configuration do not establish task infeasibility.

## Output and use

```matlab
result = RunPlanner('IndirectOCP',3);
evaluation = EvaluateResult(result,LoadCase(3));
```

The output is a **trajectory**. Its original 500 node states are retained, and each zero-speed crossing of their linear interpolation is inserted exactly, with its timestamp. Coincident floating-point endpoints are not duplicated. `a` and `omega` at a sample denote the outgoing interval's constant midpoint control; at the final sample they retain the last interval's value. The diagnostic native structure retains all original states, costates, interval controls, equation residual and nodal/midpoint extrema. The submitted state representation is linear between node states, not an exact integration of those controls. The evaluator performs its own tracking and integration; it does not accept native collision claims.

Only MATLAB and Navigation Toolbox for the shared HA initializer are needed by this planner. The indirect solver is entirely MATLAB and has no PINS, AMPL or Ipopt dependency. The common evaluator has its separate documented AMPL/Ipopt requirements. Full planner wall time includes HA, tracking, all attempted continuation solves and output conversion. Development jobs may run concurrently, so the table is not a controlled hardware timing comparison.

`TestIndirectOCP` checks canonical derivatives by complex-step differentiation, control stationarity, the complete sparse boundary-value Jacobian, a rest-to-rest straight-line problem with known 4 s time-optimal limit, and exact off-grid cusps. Release verification recomputes accepted native equations, endpoint and midpoint bounds, output fields, full-body node overlap and independent 1 ms RK4 integration under the original interval controls. These diagnostics are separate from the common executed metrics.
