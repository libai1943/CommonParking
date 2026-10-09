# DFTPAV static optimization — geometric path submission

Zhichao Han, Yuwei Wu, Tong Li, Lu Zhang, Liuao Pei, Long Xu, Chengyang Li, Changjia Ma, Chao Xu, Shaojie Shen and Fei Gao, *An Efficient Spatial-Temporal Trajectory Planner for Autonomous Vehicles in Unstructured Environments*, IEEE Transactions on Intelligent Transportation Systems 25(2), 2024, pp. 1797–1814. [DOI](https://doi.org/10.1109/TITS.2023.3315320) · [Authors' implementation](https://github.com/ZJU-FAST-Lab/Dftpav/tree/87f63ad2f67506e090b36eac38f5d247c51a330e).

`RunPlanner('DFTPAV_Path',caseId)` returns a **path**, extracted from the paper's static trajectory optimization. This scope is deliberate: the paper uses a small nonzero speed at gear changes to avoid the flatness singularity. We retain its 0.05 m/s speed in the native optimization and expose its optimized geometry to the common path executor, which independently stops at every exact cusp. The original optimized duration, speed, acceleration, steering and steering rate remain in `result.diagnostics.native_time_law`; they are **not** the submitted time law or the measured execution time. This adapter evaluates the method's optimized geometry under the common path protocol, not its original trajectory tracking performance.

This is an independent MATLAB implementation of the published static formulation. It does not incorporate the ROS system, dynamic-obstacle prediction or feedback replanning. Source parameters were checked against the linked author snapshot; no source from that GPL repository is incorporated here.

## Paper formulation

The common Hybrid A* produces the rough path and fixes the number and directions of gear runs, as in Section III-A. Each run uses uniform-duration quintic polynomial pieces. The optimization variables are interior waypoint positions, one duration per gear run, and the positions and headings of all gear changes. The minimum-integrated-squared-jerk polynomial is obtained by solving the banded interpolation system: position through fourth derivative are continuous at internal piece boundaries. Fixed endpoint position, velocity and acceleration complete the system. Gear changes have matching position/body heading, opposite velocity directions with magnitude 0.05 m/s, and zero acceleration. The body headings at the task endpoints are represented by fixed nonzero endpoint velocity vectors of the same magnitude. That endpoint convention is part of the geometric-output adapter; no claim of native rest-to-rest motion is made.

The objective is integrated squared Cartesian jerk plus total duration times `wT`, with the paper's smooth L1 penalties on sampled violations. Constraints are expressed through differential flatness: longitudinal speed, longitudinal and lateral acceleration, curvature, and the positions of **all four physical vehicle corners**. Steering-rate constraints are not added to the paper's static formulation. The common executor independently respects steering-rate limits. The author code's `getFlatState` also replaces sufficiently small endpoint speeds with a nonzero signed speed; our magnitude follows the article's 0.05 m/s rather than the later source configuration's 0.1 m/s.

For static collision avoidance, Section IV-B permits direct expansion of convex corridors in chosen directions. At every reference constraint point this implementation starts from the complete seed rectangle and expands a box along its body axes, in the order up/left/down/right, in 0.1 m steps up to 2 m per side. Each proposed expansion is tested against all complete obstacle polygons by separating axes. The resulting four half-spaces remain fixed during optimization. There is no obstacle-centre approximation and no additional obstacle inflation. Corridor generation details and growth limits are disclosed choices because the article does not prescribe a unique expansion implementation.

The piece durations are positive via equations (74)–(76). The interpolation system is written in normalized piece time; it is algebraically equivalent to the physical-time MINCO system. An analytic adjoint propagates objective and constraint gradients through the interpolation solve, both sides of each movable cusp, and the duration map. All four static feasibility terms and the full-vehicle corridor term participate in that gradient.

## Numerical realization

| Setting | Value and provenance |
|---|---|
| Gear-switch speed | 0.05 m/s, paper Section VII |
| Nominal initial piece duration | 1 s, author configuration |
| Constraint intervals per piece | 16, uniform paper-style discretization; the author code uses 32 near endpoints |
| Time / feasibility / static-obstacle weights | 500 / 2500 / 1000, author configuration |
| Lateral acceleration magnitude | 5 m/s², author configuration; other motion limits use the common vehicle |
| Smooth-L1 transition | 1e-4, equation (64) |
| Quasi-Newton solver | MATLAB `fminunc`, analytic gradients, full BFGS |
| Gradient / function / step tolerances | 1e-6 / 1e-8 / 1e-12 |
| Optimizer resource limits | 4000 iterations, 20000 function calls, 180 s |
| Native objective guard | Below 50000, following the author implementation's final-cost guard |

Full BFGS replaces the authors' limited-memory quasi-Newton implementation; neither the paper objective nor its analytic derivatives are replaced by a different trajectory NLP. A positive native optimizer flag and a finite objective below the guard are required. MATLAB's native gradient stopping test is relative to its initial scaling; the unscaled gradient infinity norm is also reported and need not be below 1e-6. The author code also accepts several resource/line-search exits; this realization reports those exits as failures. A soft-penalty optimum does **not** certify constraint satisfaction: residuals and native collisions are measured and retained separately. The positive-time map follows the printed equations without the later source's extra minimum-duration offset.

The initialization uses the common exact rest-to-rest triangular/trapezoidal speed profile to place initial waypoints and corridor samples in time; the native polynomial boundary velocities then use the above nonzero convention. Each gear run has at least two pieces and at most 80. The disclosed Hybrid A* uses 0.2 m XY cells, 5 degree heading cells, three steering samples, 0.04 m collision sampling, 0.12 m body clearance and a 180 s / 100000-expansion budget. All online initialization, corridor construction, optimization and output conversion are included in planner wall time. MATLAB Navigation and Optimization Toolboxes are required; the planner has no AMPL dependency.

## Output and independent checks

The optimized polynomial geometry is sampled uniformly in arc length separately within every gear run, with maximum spacing 0.05 m. Gear boundaries are inserted exactly. Steering comes from the analytic curvature. Arc length uses fine local Gauss quadrature followed by safeguarded inversion. Roots of the squared-speed derivative locate each piece's minimum speed; an internal flatness singularity causes an export failure instead of an arbitrary heading assignment.

`native_time_law` deliberately retains duplicate polynomial boundary timestamps and both one-sided velocities at each gear switch. It is diagnostic data, not a third standardized result kind. The standardized `result.kind` is always `path`. The common evaluator receives only that path and performs its normal path-to-trajectory conversion and whole-horizon execution optimization.

`TestDFTPAV` compares the complete analytic gradient against central differences with active motion and body penalties, verifies the minimum-jerk interpolant and C4 joins, checks the positive-time map, and checks exact cusp export. Release verification independently recomputes polynomial derivatives, quadrature, native penalties and objective, boundary states, mileage and continuous bicycle motion. Physical collision checks are observations, not a post-processing repair or a replacement success criterion.
