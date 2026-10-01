# H-OBCA

Xiaojing Zhang, Alexander Liniger, Atsushi Sakai and Francesco Borrelli, *Autonomous Parking Using Optimization-Based Collision Avoidance*, IEEE Conference on Decision and Control, 2018, pp. 4327-4332, DOI [10.1109/CDC.2018.8619433](https://doi.org/10.1109/CDC.2018.8619433). [Authors' Julia implementation](https://github.com/XiaojingGeorgeZhang/H-OBCA).

This MATLAB/AMPL implementation follows the parking paper's Algorithm 1 and Eq. (7). The underlying strong-duality reformulation is also described in Zhang, Liniger and Borrelli, *Optimization-Based Collision Avoidance*, TCST 2021, DOI [10.1109/TCST.2019.2949540](https://doi.org/10.1109/TCST.2019.2949540). The two papers are not counted as two implemented planners.

The authors' H-OBCA source is copyright (C) 2018 Alexander Liniger, Xiaojing Zhang and Atsushi Sakai, distributed under GPL-3.0-or-later. This port and its CommonParking adapters are modifications, not the authors' original release. The repository's GPL-3.0 license applies.

## Three stages

1. Hybrid A* returns a continuous-pose, collision-checked path using five steering inputs, a 0.3 m position grid, a 5-degree heading grid and Reeds-Shepp connections. The obstacle-aware 2-D shortest-path field supplies the heuristic. Numerical penalties follow the authors' public code: gear changes 10, steering changes 10, and reverse-distance penalty 0. The code's zero steering-magnitude penalty is retained. Search primitives are exact bicycle arcs and full-body collision checks use 0.04 m sampling, a documented numerical refinement over the source's 0.1 m propagation. The search limit is 100,000 expansions / 180 s.
2. The path is converted into a smooth longitudinal velocity guess obeying acceleration limits. Each obstacle's distance-dual variables are initialized separately. A geometric nearest-feature calculation seeds the auxiliary convex distance-dual optimization; the optimization is then solved and its native status checked. For separated polygons its norm inequality reaches unity at the distance optimum, as in the authors' `DualMultWS.jl`.
3. The nonlinear parking problem uses **second-order explicit midpoint Runge-Kutta** with state `(x,y,theta,v)` and inputs `(steering,acceleration)`, exactly the model structure of Eq. (7). Steering-rate bounds act on consecutive input differences. There is an independent pair of nonnegative dual vectors for **every obstacle and every node**, with unit separating-normal equality and a 0.05 m clearance constraint. Disconnected obstacles are never merged into a single polytope.

## Objective and disclosed numerical choices

The paper's objective is a linear time term plus quadratic input and input-rate terms. We retain that form, with `Q=diag(0.01,0.1)`, `Q_delta=diag(0.1,0.1)` and `kappa=10*(N-1)`, equivalent to a `10*tf` time term. The paper does not publish a numerical kappa; this is an explicit benchmark choice. The authors' code contains additional reference regularization and a quadratic time-scale term; those additions are not in Eq. (7) and are not added here.

There are 200 state nodes and 199 input intervals. The initial duration is 1.5 times the exact cusp-to-cusp longitudinal minimum time; velocity and acceleration guesses are scaled consistently. This supplies a common rule for all twelve cases because the paper's two example-specific initial time steps do not define a rule for arbitrary scenes. The paper's +/-20% time-scale bounds are retained around that initial duration. The initializer is resampled evenly in time. These are initialization/mesh choices, not extra optimization objectives.

Start and final pose and speed are constrained; final heading stays on the initializer's continuous branch. The paper treats steering as an input, so a terminal zero-steering state is **not** added to its model. For the standard output, the final steering value repeats the last interval's value, terminal acceleration is zero, and steering rate is the forward difference with zero at the last sample. This output adapter does not alter the optimized XY/heading/velocity states. The independent evaluator may therefore change the terminal approach when it enforces its own stationary endpoint steering convention.

Ipopt uses a 1e-7 tolerance, 2,000 iterations and 90 CPU seconds per solve. One restart from the candidate is allowed, following the authors' restart strategy. A candidate is not promoted to success solely because it appears feasible after a non-success native flag. Geometry and motion limits come from the shared benchmark configuration. All settings are in `+hobca/Config.m`.

The method requires MATLAB Navigation Toolbox, Optimization Toolbox (`lsqnonneg` for the geometric dual seed), and the external AMPL/Ipopt runtime. The output kind is `trajectory`.

```matlab
result = RunPlanner('H_OBCA', 3);  % 'OBCA' is an alias
evaluation = EvaluateResult(result, LoadCase(3));
PlotEvaluation(result, evaluation, LoadCase(3));
```

The reported result table distinguishes search/NLP failure, evaluator failure, collision frames, and terminal attainment. Collision constraints hold at planner nodes; dense execution safety is measured independently rather than inferred from native solver success.
