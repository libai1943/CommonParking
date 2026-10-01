# LIOM

Bai Li, Tankut Acarman, Youmin Zhang, Yakun Ouyang, Cagdas Yaman, Qi Kong, Xiang Zhong and Xiaoyan Peng, *Optimization-Based Trajectory Planning for Autonomous Parking With Irregularly Placed Obstacles: A Lightweight Iterative Framework*, IEEE Transactions on Intelligent Transportation Systems 23(8), 11970–11981, 2022 (online 2021), DOI [10.1109/TITS.2021.3109011](https://doi.org/10.1109/TITS.2021.3109011).

This MATLAB/AMPL implementation follows the paper's fault-tolerant Hybrid A*, nonlinear-constraint penalties and iterative corridor reconstruction. It is a benchmark adaptation, not the authors' original distribution or a claim to reproduce their original 115 scenarios.

## Paper mechanisms

1. Fault-tolerant Hybrid A* maintains a best-so-far state by cost to go. If its bounded search fails, an obstacle-aware 2-D A* connects that state to the target position; target heading need not be satisfied by this coarse route. The successful Hybrid A* prefix is retained. Exact rest-to-rest minimum-time longitudinal timing supplies the coarse trajectory, which is resampled uniformly in time.
2. Collision-free, axis-aligned corridors are constructed for covering-disc centres. The intermediate problem has hard variable/box bounds and linear endpoint constraints. Vehicle dynamics, the nonlinear position equations of the discs, and periodic terminal heading are quadratic penalties, **not hard nonlinear equalities**.
3. The intermediate objective is `tf + 0.01*integral(a^2+(v*omega)^2 dt) + 1e9*psi`, where `psi` is the sum of the dynamic, disc-geometry and terminal-heading penalties in Eqs. (17)–(20). The terminal term uses sine and cosine, so equivalent headings differing by 2*pi are accepted.
4. Corridors are rebuilt around each candidate. Algorithm 3 accepts only when `psi < 1e-6`; at most ten outer iterations are used. Native NLP failure is additionally checked and never relabelled as success.

## Mesh and numerical implementation

Table I specifies `N_FE=50`. Section III explicitly indexes the resampled route from 0 through `N_FE`, giving **51 states and 50 intervals**, which is the mesh used here. The first-order explicit Runge–Kutta transcription follows Section V. Dynamic residuals use `h*sum((diff(state)/h-f(state,control))^2)`; disc-geometry residuals use the corresponding time-weighted sum. The published source model states the exact quadrature/index conventions.

The entire objective is divided by 1e9 before passing it to Ipopt. This preserves the mathematical objective's minimizers and relative weights; it is numerical scaling, not penalty continuation. Ipopt uses MA97, tolerance/acceptable tolerance 1e-8, no additional NLP scaling, a monotone barrier with initial value 1e-12, 3,000 iterations and 90 CPU seconds per NLP. Initial bound perturbations and bound multipliers are 1e-12. A separate 210 s wall-clock cap also covers time spent in an individual factorization. Numerical-library threads are fixed to one. The process wrapper currently requires Windows MATLAB with .NET and an external AMPL/Ipopt/MA97 installation.

These scaled stopping tolerances and the paper's fixed large penalty can yield long-duration local solutions or numerical failures. Feasibility acceptance is checked independently from the native status; neither is a certificate of global time optimality. The final-time upper bound is 100 s and is a disclosed resource restriction. Candidate durations and failures are reported honestly.

## Vehicle and multi-disc adaptation

All vehicle geometry and motion limits come from `BenchmarkConfig`, as requested; they override the different limits in the supplied example and the paper's 0.4 m/s² acceleration setting.

The requested multi-disc extension partitions the **whole rectangular body in both directions**, placing a circumscribed circle around each rectangular cell. The partitions are `2x1, 4x2, 6x3, 8x4, 10x5, 12x6, 16x8, 20x10`, giving 2 through 200 covering discs. This is a full-body cover, not a centreline-only approximation. A cover that intersects an obstacle at a required endpoint is skipped. Otherwise its optimization is attempted; failure advances to the next finer cover. The total planner time includes all attempted covers and iterations. The first acceptable result is returned.

Corridors grow in the paper's up/left/down/right order, with a 0.1 m increment, 10 m extent and 0.01 m clearance. If an intermediate seed is obstructed, the supplied example's eight-direction relocation idea is used, with a bounded 0.16 m radial increment and 8 m maximum. This relocation rule is an implementation choice; it is not attributed to an equation in the paper. Corridor safety uses polygon–box distances and includes the full covering-disc radius.

## Initialization settings and source-code differences

The supplied LIOM example provides the Hybrid A* settings: 0.32 m / 12-degree grids, 1.5 m expansion, five steering inputs, heuristic weight 1.5, reverse and gear-change penalties 0.5, steering-change penalty 0.1, and 500 search expansions. Those values are retained. A 30 s cap and a Reeds–Shepp attempt every five expansions are implementation settings. The best-so-far selection follows the **paper's cost-to-go rule**; the later supplied code instead compares a normalized total cost. The eight-connected 2-D fallback prevents diagonal corner cutting and checks its off-grid endpoint connections.

The supplied later MATLAB example uses 200 samples, a different unweighted control-sum objective, a hard heading endpoint and changing penalty weights. Those choices are not substituted for the paper's fixed-penalty, integral-cost, periodic-heading formulation. The two-disc representation is the explicit user-requested exception, extended as described above.

`RunPlanner('LIOM', caseId)` returns a uniformly timed trajectory with XY, heading, velocity, steering, acceleration and steering rate, plus all native solve flags. Diagnostic history records skipped covers, relocated seeds, failures, outer iterations and penalty components. Public sample files retain compact diagnostics; full diagnostics are returned when the planner runs.

`TestLIOM` forces Hybrid A* failure to exercise the 2-D extension and verifies the relocated corridors geometrically. Independent release checks recompute the objective/penalty values and native constraints from the returned variables. The evaluator then performs its separate dense execution replay under the common vehicle model.
