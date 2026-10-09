# LIOM

Bai Li, Tankut Acarman, Youmin Zhang, Yakun Ouyang, Cagdas Yaman, Qi Kong, Xiang Zhong and Xiaoyan Peng, *Optimization-Based Trajectory Planning for Autonomous Parking With Irregularly Placed Obstacles: A Lightweight Iterative Framework*, IEEE Transactions on Intelligent Transportation Systems 23(8), 11970–11981, 2022 (online 2021), DOI [10.1109/TITS.2021.3109011](https://doi.org/10.1109/TITS.2021.3109011).

This MATLAB/AMPL implementation follows the paper's fault-tolerant Hybrid A*, nonlinear-constraint penalties and iterative corridor reconstruction. It is a benchmark adaptation, not the authors' original distribution or a claim to reproduce their original 115 scenarios.

## Paper mechanisms

1. Fault-tolerant Hybrid A* maintains a best-so-far state by cost to go. If its bounded search fails, an obstacle-aware 2-D A* connects that state to the target position; target heading need not be satisfied by this coarse route. The successful Hybrid A* prefix is retained. Exact rest-to-rest minimum-time longitudinal timing supplies the coarse trajectory, which is resampled uniformly in time.
2. Collision-free, axis-aligned corridors are constructed for covering-disc centres. The intermediate problem has hard variable/box bounds and linear endpoint constraints. Vehicle dynamics, the nonlinear position equations of the discs, and periodic terminal heading are quadratic penalties, **not hard nonlinear equalities**.
3. The intermediate objective is `tf + 0.01*integral(a^2+(v*omega)^2 dt) + 1e9*psi`, where `psi` is the sum of the dynamic, disc-geometry and terminal-heading penalties in Eqs. (17)–(20). The terminal term uses sine and cosine, so equivalent headings differing by 2*pi are accepted.
4. Corridors are rebuilt around each candidate. A finite inner iterate returned at an iteration/CPU limit can initialize the next outer problem only after independent real-valued, hard-bound, endpoint and penalty-consistency checks. Its unsuccessful native status is preserved. Final planner success still requires both a native `solved` status and `psi < 1e-6`; at most ten outer iterations are used. Invalid output, process failure and other native failure codes cannot continue.

## Mesh and numerical implementation

Table I specifies `N_FE=50`, or **51 states and 50 intervals**. The current benchmark configuration uses **201 states and 200 intervals**, uniformly for all cases, to refine the discretization for terminal-parking execution. This is a disclosed numerical adaptation, not the paper's Table I mesh. The completed 51-state comparison remains in the [refinement study](../../docs/LIOM_REFINEMENT.md). The first-order explicit Runge–Kutta transcription follows Section V. Dynamic residuals use `h*sum((diff(state)/h-f(state,control))^2)`; disc-geometry residuals use the corresponding time-weighted sum.

The entire objective is divided by 1e9 before passing it to Ipopt. Relative weights and mathematical minimizers are unchanged. The objective is written as local sums, with each nonlinear term depending on one node or two adjacent nodes. The previous aggregate-defined-variable expression made the AMPL/ASL Hessian pattern dense. In the independent 51-state, three-disc fixture, the Hessian pattern changes from 149,535 entries to 3,000, while eight objective comparisons agree to 1.1e-13. The aggregate expressions remain available for independent post-solve diagnostics.

Ipopt uses MA97, tolerance/acceptable tolerance 1e-8, no additional NLP scaling, a monotone barrier with initial value 1e-12, **200 iterations and 2 CPU seconds per inner NLP**. Initial bound perturbations and bound multipliers are 1e-12. A separate **10 s wall-clock cap** covers preprocessing and a solver call that does not return promptly. These caps replace the previous 3,000 iterations / 90 CPU seconds / 210 wall seconds. CPU limits are checked by Ipopt during its iteration process, so a single operation can exceed the CPU budget; the external wall cap is separate. Numerical-library threads are fixed to one. The process wrapper requires Windows MATLAB with .NET and an external AMPL/Ipopt/MA97 installation.

These scaled stopping tolerances and the paper's fixed large penalty can yield long-duration local solutions or numerical failures. Feasibility acceptance is checked independently from the native status; neither is a certificate of global time optimality. The final-time upper bound is 100 s and is a disclosed resource restriction. Candidate durations and failures are reported honestly.

## Vehicle and fixed three-disc adaptation

All vehicle geometry and motion limits come from `BenchmarkConfig`, as requested; they override the different limits in the supplied example and the paper's 0.4 m/s² acceleration setting.

The current configuration is fixed at **three discs**, with no disc-count escalation. The vehicle length is split into three equal rectangles of full vehicle width; a circumscribed circle covers each rectangle. Centres are equally spaced on the longitudinal centreline, at rear-axle-relative offsets `[-0.1475, 1.4155, 2.9785]` m. Each radius is `sqrt((4.689/6)^2+(1.942/2)^2)`, approximately 1.2464 m. Thus the three discs cover the entire physical rectangle; their diameter is not merely the vehicle width.

Both required endpoint covers are checked before search. If either intersects an obstacle or lacks the 0.01 m configured clearance, the result is `endpoint_cover_blocked`, with the start and goal clearance excesses reported separately. This rejects the **three-disc representation**, not the physical-rectangle task. No obstacles, endpoint poses or vehicle dimensions are changed. The earlier 2-to-200-disc escalation has been removed. The 201 trajectory nodes are unrelated to the count of vehicle-covering discs.

Corridors grow in the paper's up/left/down/right order, with a 0.1 m increment, 10 m extent and 0.01 m clearance. If an intermediate seed is obstructed, the supplied example's eight-direction relocation idea is used, with a bounded 0.16 m radial increment and 8 m maximum. This relocation rule is an implementation choice; it is not attributed to an equation in the paper. Corridor safety uses polygon–box distances and includes the full covering-disc radius.

## Initialization settings and source-code differences

The supplied LIOM example provides the Hybrid A* settings: 0.32 m / 12-degree grids, 1.5 m expansion, five steering inputs, heuristic weight 1.5, reverse and gear-change penalties 0.5, steering-change penalty 0.1, and 500 search expansions. Those values are retained. A 30 s cap and a Reeds–Shepp attempt every five expansions are implementation settings. The best-so-far selection follows the **paper's cost-to-go rule**; the later supplied code instead compares a normalized total cost. The eight-connected 2-D fallback prevents diagonal corner cutting and checks its off-grid endpoint connections.

The supplied later MATLAB example uses 200 samples, a different unweighted control-sum objective, a hard heading endpoint and changing penalty weights. The current implementation retains the paper's fixed penalty, integral objective and periodic heading penalty; the finer benchmark mesh and requested three-disc cover are explicit adaptations. The supplied example's 2 s inner CPU limit motivates the bounded inner solves, without copying its different objective or penalty continuation.

## Native endpoint versus executed endpoint

Native final X and Y are **hard equality constraints**. The paper's final heading is the periodic penalty in Eq. (19); it is not a hard equality in this formulation. All eight successful current native outputs have exactly zero recorded X/Y endpoint error and wrapped heading error at most 7.6e-7 rad.

The common evaluator then solves a **different, obstacle-free tracking problem whose final pose is free**, while final speed and steering are zero. Its `terminal_reached` flag refers to that independently executed trajectory. It does not say that LIOM's native hard endpoint equations were violated. Native and executed endpoint errors are both retained in the current [metrics JSON](../../results/LIOM/metrics.json) and [comparison table](../../docs/LIOM_REFINEMENT.md). The evaluation protocol and its 1 cm / 1 degree thresholds have not changed.

`RunPlanner('LIOM', caseId)` returns a uniformly timed trajectory with XY, heading, velocity, steering, acceleration and steering rate, plus all native solve flags. Diagnostics record endpoint-cover rejections, relocated seeds, failures, outer iterations and penalty components. Public sample files retain compact diagnostics; full diagnostics are returned when the planner runs.

`TestLIOM` forces Hybrid A* failure to exercise the 2-D extension, verifies three-disc relocated corridors and tests invalid inner-iterate rejection. `TestLIOMFormulation` independently compares both objective expressions and their Hessian sparsity patterns; it needs AMPL/Ipopt. Independent release checks recompute the objective/penalty values and native constraints. All twelve cases were rerun: eight solved, all eight executed without measured collisions and within terminal tolerance; cases 3 and 6 fail the endpoint-cover precheck, and cases 2 and 10 fail within the iteration budget. All failures remain in the published table.
