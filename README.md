# CommonParking

A MATLAB benchmark for motion planning in 12 static terminal-parking scenes. The accepted scene geometry and start/goal poses are frozen. Each scene requires local maneuvering; these are not long-range driving or valet-parking route tasks.

[Forty-method result index and verification](docs/RESULT_INDEX.md) · [Evaluation protocol](evaluation/README.md) · [Result contract](docs/RESULT_FORMAT.md)

[Assessment by method category](docs/METHOD_CATEGORIES.md) · [Vehicle-model alignment and outstanding gaps](docs/MODEL_ALIGNMENT.md) · [LIOM formulation and endpoint interpretation](planners/LIOM/README.md)

## Quick start

```matlab
cd('path/to/CommonParking');
SetupCommonParking;
caseData = LoadCase(1);             % inspect the scene independently
result = RunPlanner('HA_CG', 1);    % exactly two required inputs, one case
evaluation = EvaluateResult(result, caseData);
disp(evaluation.metrics);
PlotEvaluation(result, evaluation, caseData);  % optional
save('my_result.mat','result','evaluation');   % explicit output destination
```

`RunMe.m` shows the separate loading, planning, evaluation and plotting steps. `RunPlanner` itself neither saves files nor opens plots. Its only inputs are a method name and an integer case ID from 1 to 12. Algorithm settings are inside each planner's `Config.m`. Public `result` objects have exactly two possible kinds: `path` or `trajectory`; see [the complete result contract](docs/RESULT_FORMAT.md).

MATLAB R2024a was used for validation. HA+CG needs Navigation Toolbox for Reeds-Shepp and Dubins primitives. H-OBCA additionally uses Optimization Toolbox for its geometric dual seed. STC, LIOM, H-OBCA, TriangleArea, BOMP, LatticeOCP and evaluation need a separately installed AMPL/Ipopt runtime:

```matlab
setpref('CommonParking','AmplDirectory','path/to/your/ampl-and-ipopt-folder');
```

Alternatively set the `COMMONPARKING_AMPL_DIR` environment variable. Proprietary runtime executables and licenses are not redistributed. No paid service or online API is called by the benchmark. Solver scratch files go to `tempdir/CommonParking` by default, or `COMMONPARKING_WORK`; they are never written into the source repository.

BiRRT* + HC-Steer needs a one-time native MATLAB module build: run `BuildHCSteer` with a configured C++ compiler. The optional portable Zig build and external cache location are documented in [its planner README](planners/BiRRT_HC/README.md).

Lattice + OCP similarly needs `BuildLatticeGraph` once. Its validated offline primitive library and heuristic are included under the planner folder; offline preparation is excluded from online timing and can be reproduced using the documented builders.

SLiFS and DL-IAPS/PJSO additionally need MATLAB Optimization Toolbox (`quadprog`); their planners have no AMPL dependency. The common evaluator still uses the external AMPL/Ipopt runtime.

Saved results can be loaded uniformly with `[result,metrics,evaluationStatus] = LoadPublishedResult('LamirauxSmooth',2)`. Most are single MAT files. Two larger artifacts are stored in lossless binary chunks because of the publication transport's request-size limit; the loader checks their byte count and SHA-256 before loading a temporary MAT file outside the repository. No samples or precision are discarded. The stored result has the same standard structure as a fresh planner call.

Eta3 needs MATLAB Optimization Toolbox (`fmincon`) and Navigation Toolbox for its disclosed initialization adapter. Its planner has no AMPL dependency.

SE2_NMPC uses the external AMPL/Ipopt runtime (MA97), plus Navigation Toolbox for its disclosed Hybrid A* cold start. Its configuration and all model equations are contained in its planner folder.

HyperplaneOCP uses the external AMPL/Ipopt runtime (MA97). Its complete model, numerical settings and paper-to-code mapping are in [its planner folder](planners/HyperplaneOCP/README.md).

VPF uses the external AMPL/Ipopt runtime (MA97). Its complete model, numerical settings and paper-to-code mapping are in [its planner folder](planners/VPF/README.md).

RITP needs MATLAB Optimization Toolbox, plus Navigation Toolbox for Hybrid A* initialization. The planner itself has no AMPL dependency.

AnytimePSRO uses the external AMPL/Ipopt runtime with MA27. All frozen-sign constraints and the explicit-Euler model are provided in [its planner folder](planners/AnytimePSRO/README.md).

DPGrid needs a one-time C++ module build with `BuildDPGrid`; it uses base MATLAB for its planner and has no optimizer or Navigation Toolbox dependency. [Build instructions and source](planners/DPGrid/README.md).

BL_Dijkstra needs a one-time C++ module build with `BuildBLDijkstra`. It uses base MATLAB and can require several GB of memory for its full indexed search. [Source, build and resource limits](planners/BL_Dijkstra/README.md).

GraphBellman needs a one-time C++ module build with `BuildGraphBellman`. The planner uses base MATLAB and computes a value function over both direction modes on the entire grid. [Source and build instructions](planners/GraphBellman/README.md).

SmoothBiRRT needs MATLAB Navigation Toolbox for its Reeds–Shepp curves. Search, smoothing, feedback and disc collision checking are plain MATLAB in [its planner folder](planners/SmoothBiRRT/README.md).

DubinsGrid needs a one-time C++ module build with `BuildDubinsGrid` and MATLAB Optimization Toolbox for Frenet-offset SQP. Its native search has no Navigation Toolbox dependency; that toolbox is used only by the independent analytic-curve test. [Source and build instructions](planners/DubinsGrid/README.md).

BicchiTangents needs a one-time C++ module build with `BuildBicchiTangents` and otherwise uses base MATLAB. [Source, theorem scope and build instructions](planners/BicchiTangents/README.md).

TPCKC uses the external AMPL/Ipopt runtime (MA97). Its complete model, numerical settings and paper-to-code mapping are in [its planner folder](planners/TPCKC/README.md).

DFTPAV_Path needs MATLAB Navigation and Optimization Toolboxes. Its static MINCO formulation and explicit geometric-output adapter are documented in [its planner folder](planners/DFTPAV_Path/README.md). The planner does not need AMPL.

HJBA needs MATLAB Navigation and Parallel Computing Toolboxes. `BuildHJBA` optionally compiles its HJ stencil; the complete MATLAB stencil remains available. The common analytic arc helpers live in the LaumondRS folder. [Source, build and reachability assumptions](planners/HJBA/README.md).

CC_PRM uses base MATLAB after a one-time `BuildCCSteer` C++ module build. The standalone geometry test additionally uses Optimization Toolbox. [Paper construction, build instructions and roadmap adapter](planners/CC_PRM/README.md).

TDR_OBCA uses MATLAB Navigation/Optimization Toolboxes and the external AMPL/Ipopt runtime with MA97. The shared polygon-support helper is in H_OBCA. [Printed formulation and static-task adapters](planners/TDR_OBCA/README.md).

`BuildHCRSteer` compiles the licensed circle/tangent geometry and benchmark cubic-spiral implementation once. A configured MATLAB C++ compiler or Windows portable Zig compiler is supported; binaries stay outside the source tree. [Source and build instructions](planners/BiRRT_HCR/README.md).

RTR_TTS uses base MATLAB and the shared CC_PRM clothoid/distance module (`BuildCCSteer` once). It does not call the CC-family connector or roadmap search. [Algorithm, numerical choices and dependencies](planners/RTR_TTS/README.md).

Sinusoid_RTR uses base MATLAB, RTR_TTS geometric-search helpers and the shared CC_PRM distance-query module (`BuildCCSteer` once). All sinusoidal integrations and mileage inversion are implemented in MATLAB. [Paper scope and dependencies](planners/Sinusoid_RTR/README.md).

KinoDeform uses base MATLAB and the shared CC_PRM distance-query module (`BuildCCSteer` once). Its input shooting, sensitivities, deformation and integration are implemented in MATLAB. [Paper mapping and dependencies](planners/KinoDeform/README.md).

BIAGT uses base MATLAB after `BuildBIAGT` and the shared `BuildCCSteer` distance module are built once. Its complete search runs in MATLAB; the small native gateway supplies batch RS curves. [Source and build instructions](planners/BIAGT/README.md).

IndirectOCP uses MATLAB and Navigation Toolbox for its HA initialization. Its canonical state/costate solver is implemented in MATLAB without proprietary PINS or a direct NLP backend. [Formulation and disclosed numerical replacement](planners/IndirectOCP/README.md).

PointPotentialOCP needs MATLAB Optimization Toolbox for SQP. Its planner has no AMPL or Navigation Toolbox dependency. [Paper formulation and independent numerical implementation](planners/PointPotentialOCP/README.md).

## Scenes and common vehicle

The twelve case MAT files are in `cases/`; `CaseCatalog.csv` describes them and `SHA256.json` records their immutable hashes. Polygon obstacles use explicit vertex fields. Positions refer to the rear-axle midpoint; metres, seconds and radians are used. Goal orientation is compared modulo 2*pi. Tasks start and finish at rest.

| Quantity | Value |
|---|---:|
| Wheelbase | 2.8 m |
| Front / rear overhang | 0.96 / 0.929 m |
| Width / total length | 1.942 / 4.689 m |
| Forward / reverse speed magnitude | 2.5 / 2.5 m/s |
| Acceleration / braking magnitude | 1.0 / 1.0 m/s² |
| Front steering angle magnitude | 0.7 rad |
| Steering rate magnitude | 0.5 rad/s |

Geometry follows the TPCAP demonstration vehicle. Motion bounds are the explicitly adopted CommonParking rules, not an assertion that all historical TPCAP releases used identical limits. All values are centralized in `BenchmarkConfig.m`.

## Evaluation: independent dimensions, no overall score

The open evaluator screens malformed or grossly mismatched submissions, adds exact cusp-to-cusp minimum-time longitudinal timing to geometric paths, and solves an obstacle-free, whole-horizon nonlinear tracking problem. Motion limits are relaxed by **0.1%**, with unchanged vehicle geometry. It independently integrates the optimized controls and checks full-body collisions at **1 ms** frames, including the exact final time. Tracking collocation starts at 2,001 nodes and refines if numerical checks require it.

The tracker has free duration with a published time-scaling range and weak regularization. This differs from the original TPCAP fixed-timeline code. Its terminal pose is free, so final accuracy can actually be measured. [The complete evaluation specification](evaluation/README.md) states the equations, tolerances, source-code differences, interpolation, input rejection conditions, solver success handling, and all weights.

Reported dimensions are collision-frame percentage; terminal attainment (each XY error <= 1 cm and wrapped heading error <= 1 degree); executed duration; and smoothness/control-effort components. Planner computation time is reported separately. The smoothness index retains TPCAP's weights 10, 10 and 5 for the control-effort integral, steering integral and gear changes. It excludes the old time term and all competition penalties. It is not energy in joules and is not an aggregate benchmark score.

Planner success, evaluation success and terminal attainment are three different flags. A successfully evaluated path can collide or miss the goal tolerance. Invalid input or tracking failure yields a documented failure code and unavailable metrics, not an arbitrary penalty score.

## Available planners

| Name | Output | Reference | Status |
|---|---|---|---|
| [HA+CG](planners/HA_CG/README.md) (`HA_CG`) | Path | Dolgov, Thrun, Montemerlo & Diebel, IJRR 2010 | Implemented and tested on all 12 cases |
| [STC](planners/STC/README.md) (`STC`) | Trajectory | Li et al., ECC 2020; documented multi-disc extension | Implemented and tested on all 12 cases |
| [H-OBCA](planners/H_OBCA/README.md) (`H_OBCA`) | Trajectory | Zhang, Liniger, Sakai & Borrelli, CDC 2018 | Implemented and tested on all 12 cases |
| [Point-potential optimal control](planners/PointPotentialOCP/README.md) (`PointPotentialOCP`) | Trajectory | Kondak & Hommel, ICRA 2001; direct collocation, continuous point potentials and SQP with disclosed numerical adaptations | Implemented and tested on all 12 cases |
| [Indirect optimal control](planners/IndirectOCP/README.md) (`IndirectOCP`) | Trajectory | Pagot et al., Access 2023; planning component with an open canonical-equation solver | Implemented and tested on all 12 cases |
| [Bidirectional improved A-search](planners/BIAGT/README.md) (`BIAGT`) | Path | Wang, Hansen & Ahn, TCST 2024; static prioritized bidirectional search | Implemented and tested on all 12 cases |
| [Kinodynamic tree deformation](planners/KinoDeform/README.md) (`KinoDeform`) | Trajectory | Lamiraux, Ferre & Vallee, ICRA 2004; input-space trees and variational trajectory deformation | Implemented and tested on all 12 cases |
| [Sinusoidal steering + RTR](planners/Sinusoid_RTR/README.md) (`Sinusoid_RTR`) | Path | Murray & Sastry, CDC 1990 steering construction, with disclosed RTR obstacle adapter | Implemented and tested on all 12 cases |
| [RTR + TTS](planners/RTR_TTS/README.md) (`RTR_TTS`) | Path | Kiss & Tevesz, JAT 2017; RT trees and continuous-curvature local approximation | Implemented and tested on all 12 cases |
| [TDR-OBCA](planners/TDR_OBCA/README.md) (`TDR_OBCA`) | Trajectory | He et al., ACC 2021; temporal and dual initialization with printed fixed-time NLP | Implemented and tested on all 12 cases |
| [CC-Steer + directed PRM](planners/CC_PRM/README.md) (`CC_PRM`) | Path | Fraichard & Scheuer, TRO 2004; continuous-curvature connector with disclosed roadmap adapter | Implemented and tested on all 12 cases |
| [BiRRT* + HCR00-Steer](planners/BiRRT_HCR/README.md) (`BiRRT_HCR`) | Path | Banzhaf et al., IV 2018; cubic-spiral curvature-rate-continuous steering | Implemented and tested on all 12 cases |
| [HJ-guided bidirectional A*](planners/HJBA/README.md) (`HJBA`) | Path | Chi et al., TVT 2026; numerical HJ reachability and connected-state bidirectional search | Implemented and tested on all 12 cases |
| [DFTPAV static geometry](planners/DFTPAV_Path/README.md) (`DFTPAV_Path`) | Path | Han et al., TITS 2024; MINCO, movable gear changes and explicit geometric-output adapter | Implemented and tested on all 12 cases |
| [Cumulative key constraints](planners/TPCKC/README.md) (`TPCKC`) | Trajectory | Guo et al., TVT 2025; implicit-Euler NLP and cumulative vertex constraints | Implemented and tested on all 12 cases |
| [Bicchi tangent graph](planners/BicchiTangents/README.md) (`BicchiTangents`) | Path | Bicchi, Casalino & Santilli, ICRA 1995; obstacle-supported circles and tangents | Implemented and tested on all 12 cases |
| [Dubins grid + Frenet SQP](planners/DubinsGrid/README.md) (`DubinsGrid`) | Path | Siedentop et al., FAS 2015; Dubins lattice and normal-offset smoothing | Implemented and tested on all 12 cases |
| [Smooth-feedback Bi-RRT*](planners/SmoothBiRRT/README.md) (`SmoothBiRRT`) | Path | Jhang, Lian & Hao, CASE 2020; third-tree smoothing and search feedback | Implemented and tested on all 12 cases |
| [Finite-element Bellman graph](planners/GraphBellman/README.md) (`GraphBellman`) | Path | Laurini, Consolini & Locatelli, TAC 2021; Model 1 and selective Bellman updates | Implemented and tested on all 12 cases |
| [Barraquand-Latombe Dijkstra](planners/BL_Dijkstra/README.md) (`BL_Dijkstra`) | Path | Barraquand & Latombe, Algorithmica 1993; minimum-reversal indexed search | Implemented and tested on all 12 cases |
| [Backward dynamic programming](planners/DPGrid/README.md) (`DPGrid`) | Path | Schildbach & Borrelli, IV 2016; finite pose grid with continuous steering arcs | Implemented and tested on all 12 cases |
| [Anytime PSRO](planners/AnytimePSRO/README.md) (`AnytimePSRO`) | Trajectory | Chen et al., TVT 2025; frozen triangle signs, trust regions and iterative OCPs | Implemented and tested on all 12 cases |
| [Rapid iterative trajectory planning](planners/RITP/README.md) (`RITP`) | Trajectory | Li et al., RAS 2024; polynomial QPs, collision-weight iterations and physical flatness mapping | Implemented and tested on all 12 cases |
| [Virtual protection frames](planners/VPF/README.md) (`VPF`) | Trajectory | Zhang et al., IET ITS 2021; RK4 multiple shooting and iterative protection frames | Implemented and tested on all 12 cases |
| [Primal hyperplane OCP](planners/HyperplaneOCP/README.md) (`HyperplaneOCP`) | Trajectory | Fan, Murgovski & Liang, TITS 2024; polytope separating planes, interval motion bounds and time/energy OCP with disclosed search initialization | Implemented and tested on all 12 cases |
| [Orientation-aware space exploration](planners/OSEHS/README.md) (`OSEHS`) | Path | Chen, Rickert & Knoll, IV 2015; directed circles and guided heuristic search | Implemented and tested on all 12 cases |
| [Waypoint-guided two-stage RRT](planners/WGRRT/README.md) (`WGRRT`) | Path | Wang, Jha & Akemi, CASE 2017; geometric exploration, guided bi-RRT and value iteration | Implemented and tested on all 12 cases |
| [SE(2)-aware nonlinear MPC](planners/SE2_NMPC/README.md) (`SE2_NMPC`) | Trajectory | Roesmann, Makarow & Bertram, ECC 2021; static quasi-time-optimal OCP realization | Implemented and tested on all 12 cases |
| [TEB with homology exploration](planners/TEB/README.md) (`TEB`) | Trajectory | Roesmann, Hoffmann & Bertram, RAS 2017; sampled topology exploration and timed elastic bands | Implemented and tested on all 12 cases |
| [Eta3 spline optimization](planners/Eta3/README.md) (`Eta3`) | Path | Lini, Piazzi & Consolini, CDC/ECC 2011; geometric nonlinear program with disclosed online initialization | Implemented and tested on all 12 cases |
| [DL-IAPS + PJSO](planners/DL_IAPS_PJSO/README.md) (`DL_IAPS_PJSO`) | Trajectory | Zhou et al., RAL 2021; dual-loop path smoothing and piecewise-jerk speed QPs | Implemented and tested on all 12 cases |
| [C-PRM](planners/CPRM/README.md) (`CPRM`) | Path | Song & Amato, IROS 2001; lazy customized roadmap and cubic smoothing | Implemented and tested on all 12 cases |
| [Smooth canonical curves](planners/LamirauxSmooth/README.md) (`LamirauxSmooth`) | Path | Lamiraux & Laumond, TRA 2001; smooth steering and holonomic-path approximation | Implemented and tested on all 12 cases |
| [Geometric subdivision + RS](planners/LaumondRS/README.md) (`LaumondRS`) | Path | Laumond et al., TRA 1994; recursive shortest-curve approximation and shortening | Implemented and tested on all 12 cases |
| [SLiFS](planners/SLiFS/README.md) (`SLiFS`) | Trajectory | Sun et al., TITS 2022; L1 convexification within circle-centre feasible sets | Implemented and tested on all 12 cases |
| [Lattice + OCP](planners/LatticeOCP/README.md) (`LatticeOCP`) | Path | Bergman et al., TIV 2021; full-model primitives and matched-cost improvement | Implemented and tested on all 12 cases |
| [LIOM](planners/LIOM/README.md) (`LIOM`) | Trajectory | Li et al., TITS 2022; fault-tolerant initialization and multi-disc extension | Implemented and tested on all 12 cases |
| [BiRRT* + HC-Steer](planners/BiRRT_HC/README.md) (`BiRRT_HC`) | Path | Banzhaf et al., ITSC 2017; authors' HC±± geometry | Implemented and tested on all 12 cases |
| [BOMP](planners/BOMP/README.md) (`BOMP`) | Trajectory | Shi et al., IJIRA 2019; 15-node pseudospectral MAKKT | Implemented and tested on all 12 cases |
| [TriangleArea](planners/TriangleArea/README.md) (`TriangleArea`) | Trajectory | Li & Shao, KBS 2015; triangle areas with common rear-axle model and Euler transcription | Implemented and tested on all 12 cases |

Each planner has its own folder. References are named by author/title/DOI, not by a survey's numbering. This release contains 40 documented implementations and 480 recorded case outcomes. [The alphabetical result index](docs/RESULT_INDEX.md) distinguishes native success, evaluation success, terminal attainment and zero collision frames, with links to every twelve-case table and the artifact audit. Each method documents its original initialization and optimization, along with any numerical replacement or task adapter; a shared Hybrid A* initializer is not imposed on every method. Unavailable training data and undisclosed expert rules are outside the current scope.

## HA+CG: measured results

These are local MATLAB R2024a / Ipopt 3.13.4 results, not universal runtime claims. Planning times measure actual public calls including search and CG, with no cached search reuse. The public path files retain compact diagnostics; running the planner returns full stage diagnostics. All 12 planner calls and evaluator calls succeeded; 7 tracked executions satisfy the terminal tolerance and all have 0% measured collision frames. Five outputs retain the Hybrid A* path after unsuccessful safe CG modifications. Stage exit flags are preserved; this is not described as universal CG convergence.

“Time cap” indicates that the tracker's upper duration bound is active. Those durations are constrained by this explicit protocol choice and should not be interpreted as unconstrained optimal parking times. Re-run the full suite if the evaluator parameters change; do not mix tables from different protocols.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 1.450 | 0.0000 | no | 29.612 | 5.1215 | 9.9565 | 3 | 165.7801 | yes |
| 02 | yes | yes | 64.080 | 0.0000 | no | 39.677 | 4.1608 | 13.5013 | 5 | 201.6213 | yes |
| 03 | yes | yes | 1.210 | 0.0000 | yes | 27.370 | 2.9868 | 8.6961 | 2 | 126.8293 | yes |
| 04 | yes | yes | 2.675 | 0.0000 | yes | 28.426 | 2.4628 | 9.7320 | 2 | 131.9476 | yes |
| 05 | yes | yes | 4.257 | 0.0000 | yes | 23.529 | 2.2197 | 5.9837 | 2 | 92.0345 | no |
| 06 | yes | yes | 2.843 | 0.0000 | yes | 27.166 | 1.8783 | 6.8183 | 1 | 91.9663 | no |
| 07 | yes | yes | 2.307 | 0.0000 | yes | 24.015 | 2.4006 | 7.0273 | 2 | 104.2791 | yes |
| 08 | yes | yes | 1.166 | 0.0000 | no | 32.527 | 5.1720 | 12.3039 | 3 | 189.7585 | yes |
| 09 | yes | yes | 7.276 | 0.0000 | no | 35.272 | 3.8738 | 13.2112 | 3 | 185.8498 | yes |
| 10 | yes | yes | 17.152 | 0.0000 | yes | 40.550 | 2.2842 | 13.9299 | 2 | 172.1409 | no |
| 11 | yes | yes | 6.382 | 0.0000 | no | 39.610 | 2.2076 | 9.3231 | 3 | 130.3073 | no |
| 12 | yes | yes | 7.075 | 0.0000 | yes | 29.753 | 2.8080 | 7.1454 | 2 | 109.5341 | no |

Raw machine-readable data: [CSV](results/HA_CG/metrics.csv), [JSON with diagnostics](results/HA_CG/metrics.json). Each `results/HA_CG/CaseNN.mat` contains the standard `result`, measured `metrics`, and `evaluationStatus`; call `EvaluateResult` to regenerate full tracking diagnostics.

![Twelve HA+CG paths with translucent footprints](results/HA_CG/HA_CG_overview.png)

[Vector PDF, three rows by four columns](results/HA_CG/HA_CG_overview.pdf). Green denotes the start, red the goal, blue the planned path and its footprints. These show planner outputs, not the evaluator's tracked execution.

![Submitted versus tracked motion for case 1](results/HA_CG/Case01_tracking.png)

## STC: measured results

The fixed safe-corridor method uses the paper's minimum-time objective and forward Euler dynamics, with 200 planner nodes and the requested multi-disc extension. All twelve planner/evaluator calls succeeded; all twelve tracked executions meet the terminal tolerance and have 0% measured collision frames. Disc counts differ because the same deterministic refinement rule is applied in every case. Case 6 required a 16x8 partition (128 discs); unsuccessful coarser attempts are included in its computation time. No tracker time cap was active.

| Case | Planner | Evaluator | Plan time (s) | Discs | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness |
|---:|:---:|:---:|---:|---:|---:|:---:|---:|---:|---:|---:|---:|
| 01 | yes | yes | 5.144 | 8 | 0.0000 | yes | 12.064 | 11.7114 | 2.1658 | 2 | 148.7716 |
| 02 | yes | yes | 70.810 | 8 | 0.0000 | yes | 34.670 | 6.4328 | 5.8532 | 9 | 167.8604 |
| 03 | yes | yes | 1.136 | 8 | 0.0000 | yes | 9.991 | 10.6487 | 1.9882 | 1 | 131.3687 |
| 04 | yes | yes | 1.299 | 8 | 0.0000 | yes | 10.618 | 13.1083 | 1.5768 | 1 | 151.8510 |
| 05 | yes | yes | 1.473 | 8 | 0.0000 | yes | 9.786 | 10.9008 | 1.7618 | 1 | 131.6262 |
| 06 | yes | yes | 34.420 | 128 | 0.0000 | yes | 11.086 | 11.2524 | 1.9477 | 1 | 137.0006 |
| 07 | yes | yes | 6.122 | 32 | 0.0000 | yes | 9.286 | 9.3746 | 1.8632 | 1 | 117.3778 |
| 08 | yes | yes | 1.216 | 2 | 0.0000 | yes | 16.471 | 14.6103 | 3.5085 | 4 | 201.1881 |
| 09 | yes | yes | 3.753 | 18 | 0.0000 | yes | 15.418 | 9.5008 | 2.2378 | 3 | 132.3857 |
| 10 | yes | yes | 5.056 | 32 | 0.0000 | yes | 19.679 | 15.4605 | 2.9007 | 3 | 198.6114 |
| 11 | yes | yes | 11.505 | 50 | 0.0000 | yes | 24.490 | 8.5595 | 3.9311 | 6 | 154.9059 |
| 12 | yes | yes | 3.631 | 18 | 0.0000 | yes | 14.765 | 10.6106 | 2.0044 | 2 | 136.1507 |

Raw data: [CSV](results/STC/metrics.csv), [JSON](results/STC/metrics.json), and standard `result` objects in `results/STC/CaseNN.mat`. [Paper correspondence and implementation choices](planners/STC/README.md). [Dynamics and format validation](results/STC/validation.json).

<!-- results:H_OBCA -->
## H_OBCA: measured results

12/12 native solves and 12/12 evaluations succeeded; 12 executions attained the terminal tolerance, 5 had zero measured collision frames, and 5 passed both checks. The signed-distance dual formulation, objective and initialization stages use the common five-state rear bicycle with RK2, bounded steering rate, zero endpoint steering and 0.01 m physical clearance. Native discrete feasibility and the independently tracked execution remain separate outcomes. See [the formulation and numerical settings](planners/H_OBCA/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 1.0743 | 0.3804 | yes | 12.0896 | 8.3764 | 2.2662 | 2 | 116.4267 | no |
| 02 | yes | yes | 1.1768 | 0.4256 | yes | 18.0924 | 3.9708 | 3.3139 | 2 | 82.8478 | no |
| 03 | yes | yes | 0.6445 | 0 | yes | 11.5553 | 6.0562 | 2.5404 | 1 | 90.9660 | no |
| 04 | yes | yes | 0.8281 | 0 | yes | 11.4825 | 8.5404 | 1.8040 | 1 | 108.4444 | no |
| 05 | yes | yes | 0.9708 | 0 | yes | 11.5110 | 5.5432 | 2.6586 | 1 | 87.0184 | no |
| 06 | yes | yes | 2.1979 | 0.3069 | yes | 12.0537 | 6.9270 | 2.2789 | 1 | 97.0582 | no |
| 07 | yes | yes | 1.2837 | 0 | yes | 9.6729 | 6.5435 | 2.0723 | 1 | 91.1585 | no |
| 08 | yes | yes | 2.3466 | 0.4188 | yes | 15.7597 | 7.0799 | 4.0572 | 2 | 121.3706 | no |
| 09 | yes | yes | 1.9872 | 0 | yes | 15.7403 | 8.5078 | 3.3355 | 2 | 128.4327 | no |
| 10 | yes | yes | 12.8358 | 0.3702 | yes | 14.0440 | 5.3305 | 1.3578 | 1 | 71.8830 | no |
| 11 | yes | yes | 2.3544 | 0.1624 | yes | 10.4642 | 4.7811 | 2.5411 | 2 | 83.2214 | no |
| 12 | yes | yes | 6.8868 | 0.4109 | yes | 17.2796 | 5.2232 | 2.1425 | 1 | 78.6564 | no |

[CSV](results/H_OBCA/metrics.csv) · [JSON](results/H_OBCA/metrics.json) · [validation](results/H_OBCA/validation.json).
<!-- /results:H_OBCA -->

## Validation and reproducibility

```matlab
SetupCommonParking; addpath('tests');
TestCore;          % malformed data, exact cusps, asymmetric timing, angle wrapping
TestCGGradient;    % paper curvature gradient, with and without gear changes
TestDiscGeometry;  % complete body coverage and exact polygon-box distances
TestOBCADual;      % polygon distance certificates and dual equalities
TestTracker;       % known straight maneuver and independent dynamics replay
```

The CG gradient check has relative error about 1e-10. The straight-maneuver test checks an independently integrated terminal error below 1 micrometre; the executed duration differs slightly from the 4 s reference because of the published tracking regularizer. Tests also verify that multiple simultaneous obstacle overlaps count as one colliding frame, and that shifting heading by 4*pi does not change the reference. [Validation record](docs/validation.json).

The baseline paper is Dolgov et al., *Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments*, IJRR 29(5), 485–501, 2010, DOI [10.1177/0278364909359210](https://doi.org/10.1177/0278364909359210). Evaluation is adapted from the organizer's TPCAP final source and Li et al., *Online Competition of Trajectory Planning for Automated Parking: Benchmarks, Achievements, Learned Lessons, and Future Perspectives*, TIV 8(1), 2023, DOI [10.1109/TIV.2022.3228963](https://doi.org/10.1109/TIV.2022.3228963). Source code is covered by the repository's [GPL-3.0 license](LICENSE); separately installed runtimes retain their own licenses.

<!-- results:TriangleArea -->
## TriangleArea: measured results

11/12 native solves and 11/12 evaluations succeeded; 9 executions attained the terminal tolerance, 4 had zero measured collision frames, and 2 passed both checks. The supplied Triangle21 formulation uses independent corner variables, two-sided triangle-area constraints with 0.01 m^2 area excess, and 200-node explicit Euler for the common rear-axle bicycle. Multiple polygons share these same constraints. Exact task endpoints and rest states are imposed; no distance inflation is added. Native convergence and the supplied formulation's full-rectangle node collision check are both required. The common evaluator independently measures executed motion. See [the formulation and numerical settings](planners/TriangleArea/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 3.6905 | 0.2881 | yes | 11.8014 | 9.2024 | 2.6303 | 2 | 128.3272 | no |
| 02 | yes | yes | 3.7356 | 5.3404 | yes | 21.7388 | 9.6817 | 4.2990 | 2 | 149.8074 | no |
| 03 | yes | yes | 3.4104 | 0 | no | 75.1042 | 11.6552 | 3.5472 | 3 | 167.0237 | no |
| 04 | yes | yes | 20.3768 | 1.2638 | yes | 10.5222 | 12.9142 | 1.6443 | 1 | 150.5847 | no |
| 05 | yes | yes | 13.3026 | 0 | yes | 9.6206 | 10.8008 | 1.8374 | 1 | 131.3822 | no |
| 06 | yes | yes | 16.1685 | 0.4945 | yes | 10.7159 | 11.9608 | 2.0006 | 1 | 144.6135 | no |
| 07 | yes | yes | 2.1769 | 0 | yes | 8.9495 | 9.2809 | 1.7681 | 1 | 115.4894 | no |
| 08 | no | no | 76.4669 | — | — | — | — | — | — | — | — |
| 09 | yes | yes | 6.4159 | 1.0391 | yes | 22.2294 | 13.1271 | 3.1655 | 2 | 172.9258 | no |
| 10 | yes | yes | 9.0957 | 2.0333 | yes | 16.6217 | 19.1138 | 2.0723 | 2 | 221.8611 | no |
| 11 | yes | yes | 39.0367 | 0.9075 | yes | 11.6797 | 13.8245 | 1.6272 | 1 | 159.5173 | no |
| 12 | yes | yes | 17.7752 | 0 | no | 101.1044 | 14.8913 | 5.1112 | 4 | 220.0248 | no |

[CSV](results/TriangleArea/metrics.csv) · [JSON](results/TriangleArea/metrics.json) · [validation](results/TriangleArea/validation.json).
<!-- /results:TriangleArea -->

<!-- results:BOMP -->
## BOMP: measured results

The original constant initial guess, 15-node pseudospectral mesh and decreasing MAKKT relaxation are retained. Cases 5 and 9 reached native convergence; their evaluator calls succeeded, but both had replay collisions and terminal error outside tolerance. The remaining native failures are reported without fabricated scores. Independent checks verified the successful outputs' discrete model and MAKKT constraints; the dense reference steering peaks were 0.839 and 0.927 rad, exceeding the 0.7 rad node bound, and peak accelerations were 9.24 and 7.39 m/s². **The paper does not constrain acceleration, and the coarse global polynomial can overshoot between nodes.** These results concern this specified mesh and implementation. See [the paper mapping and numerical choices](planners/BOMP/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 20.6546 | — | — | — | — | — | — | — | — |
| 02 | no | no | 22.6278 | — | — | — | — | — | — | — | — |
| 03 | no | no | 9.2312 | — | — | — | — | — | — | — | — |
| 04 | no | no | 22.2790 | — | — | — | — | — | — | — | — |
| 05 | yes | yes | 12.2742 | 3.6067 | no | 48.9351 | 3.5809 | 11.4059 | 8 | 189.8681 | yes |
| 06 | no | no | 41.5762 | — | — | — | — | — | — | — | — |
| 07 | no | no | 46.1693 | — | — | — | — | — | — | — | — |
| 08 | no | no | 45.6169 | — | — | — | — | — | — | — | — |
| 09 | yes | yes | 33.3294 | 11.9561 | no | 56.0535 | 6.4203 | 15.3674 | 9 | 262.8766 | yes |
| 10 | no | no | 30.2631 | — | — | — | — | — | — | — | — |
| 11 | no | no | 77.7122 | — | — | — | — | — | — | — | — |
| 12 | no | no | 40.7007 | — | — | — | — | — | — | — | — |

[CSV](results/BOMP/metrics.csv) · [JSON](results/BOMP/metrics.json) · [validation](results/BOMP/validation.json).
<!-- /results:BOMP -->

<!-- results:BiRRT_HC -->
## BiRRT* with HC-Steer: measured results

With the paper's 6 s search budget, 11/12 planner calls found a path; case 2 exhausted its budget. All 11 evaluator calls succeeded and had 0% measured collision frames; 5 attained the terminal tolerance. Independent checks verified exact endpoints by adaptive quadrature, reverse traversal, within-gear curvature continuity and exact cusp samples. This is one fixed-seed run per case, not a 100-repeat success-rate study. See [the authors' HC geometry, MATLAB tree implementation and disclosed choices](planners/BiRRT_HC/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 7.5084 | 0 | no | 51.5246 | 3.8418 | 20.5191 | 6 | 273.6093 | yes |
| 02 | no | no | 7.2458 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 7.2373 | 0 | yes | 27.5862 | 1.6140 | 5.2778 | 3 | 83.9173 | no |
| 04 | yes | yes | 7.2338 | 0 | no | 33.9154 | 2.7317 | 8.0449 | 4 | 127.7664 | no |
| 05 | yes | yes | 7.2522 | 0 | yes | 26.1682 | 1.9335 | 7.6489 | 3 | 110.8237 | no |
| 06 | yes | yes | 7.4283 | 0 | yes | 35.2626 | 2.5366 | 8.4139 | 5 | 134.5049 | no |
| 07 | yes | yes | 7.2523 | 0 | yes | 101.8182 | 5.1464 | 30.7694 | 13 | 424.1576 | no |
| 08 | yes | yes | 7.2396 | 0 | no | 65.1163 | 3.7811 | 23.2469 | 7 | 305.2800 | no |
| 09 | yes | yes | 7.2406 | 0 | no | 50.4504 | 2.7957 | 16.4369 | 4 | 212.3264 | no |
| 10 | yes | yes | 7.2257 | 0 | yes | 114.2857 | 4.3083 | 30.5731 | 15 | 423.8148 | no |
| 11 | yes | yes | 7.2457 | 0 | no | 38.0353 | 2.5585 | 6.9878 | 5 | 120.4630 | no |
| 12 | yes | yes | 7.2289 | 0 | no | 57.7320 | 2.8714 | 15.9703 | 8 | 228.4171 | no |

[CSV](results/BiRRT_HC/metrics.csv) · [JSON](results/BiRRT_HC/metrics.json) · [validation](results/BiRRT_HC/validation.json).
<!-- /results:BiRRT_HC -->

<!-- results:LIOM -->
## LIOM: measured results

LIOM uses exactly three uniformly spaced covering discs and 201 states (200 intervals), with no disc-count escalation. The paper's fault-tolerant initialization, fixed 1e9 penalty and iterative corridor reconstruction are retained. An equivalent sparse objective expression and short inner solves permit useful outer iterations. 8/12 calls passed both the native solve flag and the paper infeasibility threshold; all eight executions had zero measured collision frames and attained the terminal tolerance. Cases 3 and 6 fail the fixed-cover endpoint precheck; cases 2 and 10 exhaust the outer budget. Native X/Y endpoint errors are zero in every accepted solution. The terminal column below describes the independently tracked execution, not the native hard endpoint constraints. Independent checks recomputed penalty components, objective, corridor geometry and hard bounds. See [the source mapping and settings](planners/LIOM/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Executed terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 3.1443 | 0 | yes | 12.1781 | 11.0293 | 1.8077 | 2 | 138.3707 | no |
| 02 | no | no | 9.2176 | — | — | — | — | — | — | — | — |
| 03 | no | no | 0.0553 | — | — | — | — | — | — | — | — |
| 04 | yes | yes | 2.2070 | 0 | yes | 10.8358 | 11.5626 | 1.4164 | 1 | 134.7901 | no |
| 05 | yes | yes | 2.4297 | 0 | yes | 11.1283 | 8.0685 | 2.1016 | 1 | 106.7003 | no |
| 06 | no | no | 0.0162 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 4.0850 | 0 | yes | 11.1047 | 5.5423 | 2.5576 | 1 | 85.9991 | no |
| 08 | yes | yes | 3.6125 | 0 | yes | 15.3511 | 12.5474 | 2.7820 | 3 | 168.2938 | no |
| 09 | yes | yes | 5.5034 | 0 | yes | 23.2299 | 8.0120 | 5.0419 | 4 | 150.5391 | no |
| 10 | no | no | 12.5997 | — | — | — | — | — | — | — | — |
| 11 | yes | yes | 4.6869 | 0 | yes | 13.8963 | 12.0408 | 3.1178 | 3 | 166.5851 | no |
| 12 | yes | yes | 4.9977 | 0 | yes | 13.7126 | 11.4099 | 2.3961 | 2 | 148.0598 | no |

[CSV](results/LIOM/metrics.csv) · [JSON](results/LIOM/metrics.json) · [validation](results/LIOM/validation.json).
<!-- /results:LIOM -->

<!-- results:LatticeOCP -->
## Lattice planning with matched optimal control: measured results

The complete-model lattice uses 480 optimized motion primitives, the matched-cost free-space heuristic, and the same five-state model/cost in the final multiphase OCP. Cases 4 and 8 passed native optimization and evaluation, had 0% measured collision frames, and attained the terminal tolerance. Seven other cases are blocked by the paper's conservative three-circle vehicle/obstacle covers at a required endpoint; three had no route in the configured lattice graph. These failures are not claims that the original rectangular-vehicle tasks are infeasible. All 480 primitives passed independent ODE integration; known-feasible one-phase and forward/reverse OCPs also converged before release. See [the original formulation, off-grid endpoint treatment, circle covers and Ipopt implementation](planners/LatticeOCP/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 2.1073 | — | — | — | — | — | — | — | — |
| 02 | no | no | 1.3462 | — | — | — | — | — | — | — | — |
| 03 | no | no | 1.3242 | — | — | — | — | — | — | — | — |
| 04 | yes | yes | 124.2027 | 0 | yes | 12.1950 | 10.8222 | 0.3795 | 1 | 117.0165 | no |
| 05 | no | no | 1.4035 | — | — | — | — | — | — | — | — |
| 06 | no | no | 1.3421 | — | — | — | — | — | — | — | — |
| 07 | no | no | 1.3574 | — | — | — | — | — | — | — | — |
| 08 | yes | yes | 116.9322 | 0 | yes | 18.2738 | 17.1169 | 1.5684 | 3 | 201.8535 | no |
| 09 | no | no | 2.1345 | — | — | — | — | — | — | — | — |
| 10 | no | no | 1.4686 | — | — | — | — | — | — | — | — |
| 11 | no | no | 1.6476 | — | — | — | — | — | — | — | — |
| 12 | no | no | 1.4698 | — | — | — | — | — | — | — | — |

[CSV](results/LatticeOCP/metrics.csv) · [JSON](results/LatticeOCP/metrics.json) · [validation](results/LatticeOCP/validation.json).
<!-- /results:LatticeOCP -->

<!-- results:SLiFS -->
## SLiFS: measured results

10/12 native solves and 10/12 evaluations succeeded; 9 executions attained the terminal tolerance, 7 had zero measured collision frames, and 6 passed both checks. The L1 successive-linearization QPs, circle-centre feasible sets and first-step perturbations use 200 states, three full-body covering circles and 0.01 m optimization clearance. The fixed QP budget is 300; lack of convergence remains failure. Hybrid A* supplies the initial path. Analytic derivatives, physical and affine circle distances, discrete dynamics, endpoints, controls and swept footprints are independently checked. See [the formulation and numerical settings](planners/SLiFS/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 3.7534 | 0 | yes | 15.9581 | 4.3133 | 1.9252 | 2 | 72.3849 | no |
| 02 | yes | yes | 49.9279 | 0 | no | 56.0965 | 3.5686 | 5.4422 | 12 | 150.1075 | no |
| 03 | no | no | 0.0334 | — | — | — | — | — | — | — | — |
| 04 | yes | yes | 2.4252 | 0 | yes | 23.4879 | 1.3023 | 1.9107 | 1 | 37.1294 | no |
| 05 | yes | yes | 3.5353 | 0 | yes | 23.5809 | 1.8216 | 2.8930 | 3 | 62.1466 | no |
| 06 | no | no | 0.0279 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 12.9428 | 0 | yes | 13.0500 | 5.4335 | 2.2924 | 2 | 87.2594 | no |
| 08 | yes | yes | 3.6193 | 0 | yes | 14.1425 | 10.0136 | 2.6862 | 3 | 141.9978 | no |
| 09 | yes | yes | 6.5819 | 0.0631 | yes | 25.3497 | 5.4061 | 3.7124 | 3 | 106.1858 | no |
| 10 | yes | yes | 24.0729 | 0.1597 | yes | 30.6831 | 8.3110 | 2.9452 | 4 | 132.5619 | no |
| 11 | yes | yes | 4.3993 | 1.7276 | yes | 23.8467 | 4.5864 | 3.8290 | 5 | 109.1543 | no |
| 12 | yes | yes | 21.1027 | 0 | yes | 17.1793 | 5.2529 | 2.0343 | 1 | 77.8717 | no |

[CSV](results/SLiFS/metrics.csv) · [JSON](results/SLiFS/metrics.json) · [validation](results/SLiFS/validation.json).
<!-- /results:SLiFS -->

<!-- results:LaumondRS -->
## Geometric subdivision with Reeds–Shepp shortening: measured results

All 12 calls produced collision-free bounded-curvature paths and all 12 execution optimizations succeeded; 4 executions attained the terminal tolerance. Independent continuous primitive checks and adaptive ODE quadrature verified every submitted path. Execution can still deviate because steering jumps are allowed in these paths. In particular, case 11 retained a long, highly cusped path after the configured random shortening budget, with a 203.18 s reference and a 609.54 s tracked execution; its replay collision percentage is 12.64%. These are measured limitations, not filtered-out trials. See [the three paper stages, exact swept-arc tests and finite resource bounds](planners/LaumondRS/README.md).

Planning times include geometric search, recursive connection and shortening. MATLAB uses its default numerical-library thread setting for this geometric method. Other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 2.1640 | 0 | no | 31.0373 | 6.7957 | 10.2758 | 4 | 190.7147 | yes |
| 02 | yes | yes | 3.8423 | 3.7470 | no | 39.1767 | 8.4329 | 11.4596 | 7 | 233.9252 | yes |
| 03 | yes | yes | 2.6962 | 0.7226 | yes | 26.2911 | 2.3204 | 9.1769 | 2 | 124.9733 | no |
| 04 | yes | yes | 2.3914 | 0 | no | 27.8229 | 5.5338 | 9.9944 | 2 | 165.2816 | yes |
| 05 | yes | yes | 2.3922 | 0.0512 | yes | 25.3905 | 2.5119 | 7.8938 | 2 | 114.0569 | yes |
| 06 | yes | yes | 3.0917 | 19.8088 | no | 30.4348 | 2.1194 | 7.7880 | 3 | 114.0747 | no |
| 07 | yes | yes | 2.1959 | 0 | yes | 24.0146 | 2.4027 | 7.0159 | 2 | 104.1860 | yes |
| 08 | yes | yes | 4.4865 | 0 | no | 35.0067 | 4.5424 | 12.9253 | 4 | 194.6774 | yes |
| 09 | yes | yes | 2.9762 | 1.9080 | no | 60.3744 | 12.9290 | 16.8303 | 11 | 352.5922 | yes |
| 10 | yes | yes | 4.0197 | 3.6859 | no | 53.3646 | 8.2902 | 12.7868 | 10 | 260.7697 | yes |
| 11 | yes | yes | 3.1908 | 12.6397 | no | 609.5405 | 170.3279 | 100.5022 | 193 | 3673.3008 | yes |
| 12 | yes | yes | 2.9520 | 8.6349 | yes | 31.2204 | 4.7115 | 9.3481 | 2 | 150.5962 | yes |

[CSV](results/LaumondRS/metrics.csv) · [JSON](results/LaumondRS/metrics.json) · [validation](results/LaumondRS/validation.json).
<!-- /results:LaumondRS -->

<!-- results:LamirauxSmooth -->
## Smooth canonical-curve planning: measured results

All 12 calls produced native collision-free continuous-curvature paths. Eleven passed the common evaluator, with zero measured collision frames and terminal attainment in every one. Case 8 retained a path longer than the evaluator's published 1,000 m resource limit and was rejected by its initial screen; its metrics are unavailable. Case 2 still required about 482 s of tracked execution. These long, highly maneuvering solutions are retained, not replaced by another planner. Independent checks verified analytic derivatives, interval bounds, regularity, curvature, continuous footprint clearance, joins, cusps and mileage inversion. See [the paper construction and disclosed finite numerical interpretations](planners/LamirauxSmooth/README.md).

Planning times include all online construction, search and smoothing stages. MATLAB uses its default numerical-library thread setting for this geometric method. Other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 64.6100 | 0 | yes | 31.8726 | 3.2445 | 6.3220 | 5 | 120.6653 | no |
| 02 | yes | yes | 57.1464 | 0 | yes | 481.9685 | 113.7583 | 14.1476 | 257 | 2564.0592 | no |
| 03 | yes | yes | 43.2083 | 0 | yes | 18.0516 | 3.0137 | 3.5833 | 1 | 70.9693 | no |
| 04 | yes | yes | 33.5809 | 0 | yes | 17.8124 | 3.8916 | 2.3381 | 1 | 67.2966 | no |
| 05 | yes | yes | 62.5560 | 0 | yes | 17.8751 | 3.0186 | 3.4052 | 2 | 74.2381 | no |
| 06 | yes | yes | 48.8609 | 0 | yes | 46.7713 | 8.4311 | 5.0353 | 9 | 179.6640 | no |
| 07 | yes | yes | 62.9153 | 0 | yes | 24.5837 | 5.3477 | 4.2619 | 3 | 111.0957 | no |
| 08 | yes | no | 105.6687 | — | — | — | — | — | — | — | — |
| 09 | yes | yes | 58.5063 | 0.9865 | yes | 20.8809 | 5.3918 | 2.6629 | 3 | 95.5467 | no |
| 10 | yes | yes | 64.5928 | 0 | yes | 26.5249 | 6.2015 | 7.2192 | 2 | 144.2076 | no |
| 11 | yes | yes | 63.4632 | 0.9897 | yes | 16.5684 | 2.6187 | 3.2884 | 4 | 79.0710 | no |
| 12 | yes | yes | 63.3822 | 0 | yes | 25.7143 | 4.9260 | 4.0595 | 3 | 104.8552 | no |

[CSV](results/LamirauxSmooth/metrics.csv) · [JSON](results/LamirauxSmooth/metrics.json) · [validation](results/LamirauxSmooth/validation.json).
<!-- /results:LamirauxSmooth -->

<!-- results:CPRM -->
## Customizable probabilistic roadmaps: measured results

Seven of the 12 fixed-seed calls found a validated roadmap path. All seven execution optimizations succeeded with zero measured collision frames; four attained the terminal tolerance. The five native query failures remain failures. The planner rebuilds the roadmap for every call, so these timings include construction and do not measure multi-query reuse. The paper's cubic smoothing is attempted on eligible sections; retained circular sections and endpoint attachments can have curvature jumps. Independent checks verified tangent fillets, spline basis conversion and curvature extrema, continuous footprint clearance, exact cusps, endpoint poses and mileage quadrature. See [the two roadmap levels, lazy query, partial smoothing and disclosed adapters](planners/CPRM/README.md).

Planning times include all online construction, search and smoothing stages. MATLAB uses its default numerical-library thread setting for this geometric method. Other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 5.6941 | — | — | — | — | — | — | — | — |
| 02 | no | no | 3.9149 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 4.1164 | 0 | yes | 39.1141 | 3.4155 | 9.3815 | 3 | 142.9694 | no |
| 04 | yes | yes | 3.8328 | 0 | yes | 59.5745 | 3.9753 | 10.9954 | 6 | 179.7067 | no |
| 05 | no | no | 3.8658 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 3.6858 | 0 | no | 47.6390 | 4.1116 | 13.3332 | 5 | 199.4480 | yes |
| 07 | yes | yes | 4.1042 | 0 | yes | 72.7273 | 4.1483 | 11.3261 | 7 | 189.7439 | no |
| 08 | yes | yes | 3.6303 | 0 | no | 52.2235 | 4.4123 | 12.5614 | 5 | 194.7377 | yes |
| 09 | no | no | 3.9503 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 4.9054 | 0 | yes | 58.9474 | 3.5718 | 17.7339 | 5 | 238.0568 | no |
| 11 | yes | yes | 3.7912 | 0 | no | 115.7534 | 4.5756 | 22.1000 | 16 | 346.7559 | no |
| 12 | no | no | 3.3167 | — | — | — | — | — | — | — | — |

[CSV](results/CPRM/metrics.csv) · [JSON](results/CPRM/metrics.json) · [validation](results/CPRM/validation.json).
<!-- /results:CPRM -->

<!-- results:DL_IAPS_PJSO -->
## DL_IAPS_PJSO: measured results

12/12 native solves and 12/12 evaluations succeeded; 12 executions attained the terminal tolerance, 12 had zero measured collision frames, and 12 passed both checks. The dual-loop path QPs and piecewise-jerk speed QP use common curvature and longitudinal limits, verified steering-rate adaptation and explicit steering transitions at rest. Native discrete geometry remains an approximation; the common execution evaluator measures collisions and terminal attainment independently. See [the formulation and numerical settings](planners/DL_IAPS_PJSO/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 1.0803 | 0 | yes | 34.2439 | 1.3168 | 7.1676 | 2 | 94.8440 | no |
| 02 | yes | yes | 11.2452 | 0 | yes | 39.6255 | 1.5092 | 6.6828 | 3 | 96.9205 | no |
| 03 | yes | yes | 2.2728 | 0 | yes | 30.3660 | 0.9937 | 5.1870 | 1 | 66.8062 | no |
| 04 | yes | yes | 2.7807 | 0 | yes | 31.5251 | 1.1777 | 6.3547 | 1 | 80.3231 | no |
| 05 | yes | yes | 2.5751 | 0 | yes | 28.8292 | 2.0138 | 4.8261 | 1 | 73.3987 | no |
| 06 | yes | yes | 4.7963 | 0 | yes | 113.2194 | 0.3993 | 16.0991 | 1 | 169.9840 | no |
| 07 | yes | yes | 13.2676 | 0 | yes | 26.9488 | 0.7958 | 5.0220 | 1 | 63.1783 | no |
| 08 | yes | yes | 0.6628 | 0 | yes | 43.8840 | 1.5411 | 9.1783 | 3 | 122.1935 | no |
| 09 | yes | yes | 17.4988 | 0 | yes | 32.7273 | 1.4517 | 3.5948 | 1 | 55.4651 | no |
| 10 | yes | yes | 17.7647 | 0 | yes | 40.7040 | 4.8716 | 4.2167 | 3 | 105.8827 | no |
| 11 | yes | yes | 2.0917 | 0 | yes | 29.5834 | 0.6667 | 6.0942 | 2 | 77.6092 | no |
| 12 | yes | yes | 11.8178 | 0 | yes | 42.8958 | 1.0119 | 7.2459 | 2 | 92.5779 | no |

[CSV](results/DL_IAPS_PJSO/metrics.csv) · [JSON](results/DL_IAPS_PJSO/metrics.json) · [validation](results/DL_IAPS_PJSO/validation.json).
<!-- /results:DL_IAPS_PJSO -->

<!-- results:Eta3 -->
## Multi-optimization of eta3 splines: measured results

The simplified seventh-degree eta3 curve family and its 8h-4-variable nonlinear program produced 8/12 native successes; 8 execution optimizations succeeded and 8 attained the terminal tolerance. The article's 100-interval parameter mesh, objective weights and curvature-derivative limit are retained. An explicitly disclosed Hybrid A* adapter supplies the maneuver sequence and initial cusp estimates in place of unavailable offline lookup tables. Full-body collision and curvature constraints are sampled, so between-node or execution violations can remain and are reported. Independent checks verified polynomial endpoint geometry, native constraints/objective, cusp curvature, exact mileage sampling and adaptive quadrature. See [the complete paper mapping and finite numerical choices](planners/Eta3/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 8.7410 | 0 | yes | 31.6879 | 1.2656 | 7.5693 | 2 | 98.3495 | yes |
| 02 | no | no | 131.3805 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 4.9868 | 0 | yes | 17.2725 | 2.4074 | 4.5224 | 1 | 74.2979 | no |
| 04 | yes | yes | 6.1550 | 0.9240 | yes | 26.8398 | 1.2483 | 6.6922 | 1 | 84.4051 | no |
| 05 | yes | yes | 4.9347 | 0 | yes | 11.9759 | 5.2095 | 2.7239 | 1 | 84.3345 | no |
| 06 | yes | yes | 7.0705 | 0 | yes | 18.0864 | 2.9304 | 3.6381 | 1 | 70.6848 | no |
| 07 | no | no | 4.3623 | — | — | — | — | — | — | — | — |
| 08 | yes | yes | 5.9058 | 0 | yes | 34.6574 | 1.3320 | 10.1009 | 2 | 124.3293 | yes |
| 09 | no | no | 98.8114 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 19.1833 | 0.5676 | yes | 19.9088 | 6.4715 | 5.8381 | 2 | 133.0960 | no |
| 11 | no | no | 24.1854 | — | — | — | — | — | — | — | — |
| 12 | yes | yes | 53.8764 | 1.5570 | yes | 20.9361 | 3.1975 | 3.2891 | 1 | 69.8659 | no |

[CSV](results/Eta3/metrics.csv) · [JSON](results/Eta3/metrics.json) · [validation](results/Eta3/validation.json).
<!-- /results:Eta3 -->

<!-- results:TEB -->
## TEB: measured results

12/12 native solves and 12/12 evaluations succeeded; 11 executions attained the terminal tolerance, 8 had zero measured collision frames, and 7 passed both checks. The topology exploration and sparse LM elastic-band optimizer use the common five-state rear bicycle, explicit nodal speed/steering, exact rest endpoints, 0.01 m footprint clearance and fixed soft penalties. Native completion is not a hard-feasibility certificate; independent checks report residuals and the unchanged evaluator measures executed collisions and terminal attainment. See [the formulation and numerical settings](planners/TEB/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 2.5000 | 0 | yes | 18.3134 | 5.0979 | 4.2078 | 4 | 113.0577 | no |
| 02 | yes | yes | 3.0927 | 0 | yes | 24.3355 | 2.5142 | 5.3586 | 2 | 88.7275 | no |
| 03 | yes | yes | 2.4067 | 0 | yes | 14.6191 | 3.9966 | 3.3273 | 1 | 78.2391 | no |
| 04 | yes | yes | 2.4178 | 0 | yes | 17.5705 | 5.2807 | 4.7829 | 2 | 110.6364 | no |
| 05 | yes | yes | 1.9784 | 0 | yes | 12.6987 | 4.7077 | 2.5551 | 1 | 77.6284 | no |
| 06 | yes | yes | 5.0119 | 0.4940 | yes | 14.9789 | 5.0005 | 2.8258 | 1 | 83.2635 | no |
| 07 | yes | yes | 2.3702 | 0 | no | 12.9148 | 3.1598 | 3.1964 | 1 | 68.5622 | no |
| 08 | yes | yes | 2.5440 | 0 | yes | 18.5189 | 5.9806 | 5.2765 | 2 | 122.5710 | no |
| 09 | yes | yes | 3.7505 | 0.3394 | yes | 25.0412 | 4.7025 | 5.6909 | 3 | 118.9335 | no |
| 10 | yes | yes | 6.2344 | 0.2679 | yes | 15.6755 | 5.2194 | 1.5618 | 1 | 72.8122 | no |
| 11 | yes | yes | 2.4758 | 0 | yes | 14.4925 | 2.3480 | 3.7089 | 2 | 70.5683 | no |
| 12 | yes | yes | 5.8910 | 0.7937 | yes | 15.3690 | 6.4148 | 2.5337 | 2 | 99.4843 | no |

[CSV](results/TEB/metrics.csv) · [JSON](results/TEB/metrics.json) · [validation](results/TEB/validation.json).
<!-- /results:TEB -->

<!-- results:SE2_NMPC -->
## SE2_NMPC: measured results

11/12 native solves and 11/12 evaluations succeeded; 11 executions attained the terminal tolerance, 11 had zero measured collision frames, and 11 passed both checks. The wrapped-angle Crank-Nicolson OCP uses common nodal speed/steering, exact rest endpoints and 0.01 m physical clearance. Shared interval distance-dual normals and a corner-rotation remainder bound cover the submitted linear pose reference between nodes. The uniform budget is 30 CPU seconds; native and execution outcomes remain separate. See [the formulation and numerical settings](planners/SE2_NMPC/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 9.4775 | 0 | yes | 10.6016 | 14.2443 | 1.8748 | 2 | 171.1909 | no |
| 02 | yes | yes | 24.2263 | 0 | yes | 12.8841 | 14.3825 | 2.3022 | 3 | 181.8477 | no |
| 03 | yes | yes | 0.9682 | 0 | yes | 9.5519 | 12.1499 | 1.9870 | 1 | 146.3688 | no |
| 04 | yes | yes | 4.5111 | 0 | yes | 10.1452 | 14.3122 | 1.5792 | 1 | 163.9142 | no |
| 05 | yes | yes | 3.0969 | 0 | yes | 9.2296 | 12.1235 | 1.7691 | 1 | 143.9257 | no |
| 06 | yes | yes | 3.2333 | 0 | yes | 10.3363 | 13.2711 | 1.9382 | 1 | 157.0934 | no |
| 07 | yes | yes | 6.7170 | 0 | yes | 8.6795 | 9.9878 | 1.7330 | 1 | 122.2076 | no |
| 08 | yes | yes | 25.6177 | 0 | yes | 12.5537 | 14.8534 | 2.6778 | 3 | 190.3116 | no |
| 09 | yes | yes | 5.2437 | 0 | yes | 15.4150 | 18.2425 | 2.7177 | 3 | 224.6022 | no |
| 10 | yes | yes | 6.4470 | 0 | yes | 11.2932 | 16.0069 | 1.3532 | 1 | 178.6009 | no |
| 11 | no | no | 30.6373 | — | — | — | — | — | — | — | — |
| 12 | yes | yes | 9.0013 | 0 | yes | 11.7906 | 16.3017 | 1.8678 | 2 | 191.6952 | no |

[CSV](results/SE2_NMPC/metrics.csv) · [JSON](results/SE2_NMPC/metrics.json) · [validation](results/SE2_NMPC/validation.json).
<!-- /results:SE2_NMPC -->

<!-- results:WGRRT -->
## Waypoint-guided two-stage RRT: measured results

The geometric waypoint stage, accumulated bidirectional RRT graph and Bellman value iteration produced 11/12 native paths; 11 execution optimizations succeeded and 6 attained the terminal tolerance. The paper's neighbor radius 6, three RS candidates and 100 new nodes per waypoint connection are retained. **All submitted paths passed continuous full-body collision checks, but six tracked executions have nonzero collision percentages.** Discontinuous steering and the common execution model can change the path actually followed; native safety is not executable safety. The native failure and all execution deviations remain in the table. Independent validation checks every successful graph edge by adaptive quadrature and collision geometry, compares the recovered path with a separate graph shortest-distance algorithm, and verifies exact cusps and output mileage. See [the paper stages and disclosed sampling, finite-budget and shortening choices](planners/WGRRT/README.md).

Planning times include all online construction, search and smoothing stages. MATLAB uses its default numerical-library thread setting for this geometric method. Other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 8.2239 | 9.8538 | no | 32.9608 | 7.3123 | 10.4744 | 5 | 202.8670 | yes |
| 02 | no | no | 7.0015 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 10.1841 | 4.2094 | yes | 26.4151 | 1.9021 | 8.6995 | 2 | 116.0160 | no |
| 04 | yes | yes | 12.8365 | 0 | no | 28.7539 | 3.6026 | 10.4510 | 2 | 150.5358 | yes |
| 05 | yes | yes | 14.8588 | 0.3484 | yes | 25.5462 | 2.5076 | 7.9800 | 2 | 114.8757 | yes |
| 06 | yes | yes | 14.0808 | 4.6514 | yes | 41.7498 | 5.3610 | 12.4327 | 6 | 207.9363 | yes |
| 07 | yes | yes | 13.1389 | 0 | yes | 25.1059 | 2.4007 | 7.2776 | 3 | 111.7824 | yes |
| 08 | yes | yes | 4.3494 | 0 | no | 32.4924 | 4.4350 | 11.8710 | 3 | 178.0601 | yes |
| 09 | yes | yes | 3.6158 | 9.0077 | no | 48.4800 | 10.7602 | 14.3295 | 11 | 305.8973 | yes |
| 10 | yes | yes | 5.2387 | 0 | yes | 51.7827 | 4.8036 | 18.8342 | 7 | 271.3783 | yes |
| 11 | yes | yes | 7.0216 | 0 | no | 72.0248 | 9.1965 | 25.5299 | 12 | 407.2640 | yes |
| 12 | yes | yes | 6.0132 | 8.3296 | yes | 31.3922 | 4.5156 | 9.3189 | 2 | 148.3452 | yes |

[CSV](results/WGRRT/metrics.csv) · [JSON](results/WGRRT/metrics.json) · [validation](results/WGRRT/validation.json).
<!-- /results:WGRRT -->

<!-- results:OSEHS -->
## Orientation-aware space exploration guided search: measured results

The directed-circle route and independent circle-guided kinematic search produced 10/12 native paths; 10 execution optimizations succeeded and 6 attained the terminal tolerance. Cases 2 and 5 exhausted the configured search refinements. All successful paths passed continuous full-body collision checks, while the tracked execution of case 11 had 1.98% collision frames. The 2015 six-primitive model, radius-dependent search resolution and direction-aware penalties are retained; implementation choices omitted from the article are documented. Independent checks recompute the circle geometry and search objective, integrate every primitive, and verify exact endpoint poses, cusps and mileage spacing. See [the paper mapping, finite budgets and analytic goal expansion](planners/OSEHS/README.md).

Planning times include all online construction, search and any smoothing stages. MATLAB uses its default numerical-library thread setting for this geometric method. Other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 8.4350 | 0 | no | 29.5861 | 6.5967 | 9.9436 | 3 | 180.4026 | yes |
| 02 | no | no | 183.9453 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 13.2684 | 0 | yes | 23.3247 | 2.2704 | 4.8978 | 1 | 76.6813 | no |
| 04 | yes | yes | 7.8912 | 0 | no | 36.3722 | 2.9306 | 11.3251 | 4 | 162.5567 | yes |
| 05 | no | no | 184.1384 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 35.3410 | 0 | yes | 49.6984 | 5.7551 | 11.7263 | 7 | 209.8145 | yes |
| 07 | yes | yes | 4.2925 | 0 | yes | 48.4859 | 4.7186 | 12.9320 | 7 | 211.5052 | yes |
| 08 | yes | yes | 12.4761 | 0 | no | 42.9539 | 2.8783 | 16.3579 | 5 | 217.3618 | no |
| 09 | yes | yes | 8.8421 | 0 | no | 36.1165 | 4.1585 | 11.5107 | 3 | 171.6919 | yes |
| 10 | yes | yes | 13.0423 | 0 | yes | 67.9864 | 4.7497 | 17.0197 | 8 | 257.6932 | yes |
| 11 | yes | yes | 4.0159 | 1.9794 | yes | 37.3333 | 2.5401 | 8.8913 | 5 | 139.3146 | no |
| 12 | yes | yes | 18.8324 | 0 | yes | 36.1722 | 4.3584 | 8.1884 | 2 | 135.4672 | yes |

[CSV](results/OSEHS/metrics.csv) · [JSON](results/OSEHS/metrics.json) · [validation](results/OSEHS/validation.json).
<!-- /results:OSEHS -->

<!-- results:HyperplaneOCP -->
## HyperplaneOCP: measured results

12/12 native solves and 12/12 evaluations succeeded; 12 executions attained the terminal tolerance, 12 had zero measured collision frames, and 12 passed both checks. The primal separating-plane RK4 OCP uses the common five-state car, a bounded search seed, unit normals and shared interval planes with bounds on corner rotation and physical corner acceleration. The fixed 60 CPU-second NLP budget, 200 intervals and original time/energy objective are uniform across cases. Native constraints, linearly interpolated reference geometry and independent execution outcomes are checked separately. See [the formulation and numerical settings](planners/HyperplaneOCP/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 7.6359 | 0 | yes | 57.0365 | 0.1581 | 7.1863 | 2 | 83.4439 | no |
| 02 | yes | yes | 3.3553 | 0 | yes | 97.5998 | 0.2274 | 23.4210 | 4 | 256.4840 | no |
| 03 | yes | yes | 3.3735 | 0 | yes | 47.0475 | 0.1294 | 2.8646 | 1 | 34.9403 | no |
| 04 | yes | yes | 1.4508 | 0 | yes | 49.4143 | 0.1524 | 1.2257 | 1 | 18.7810 | no |
| 05 | yes | yes | 2.0555 | 0 | yes | 55.2663 | 0.1189 | 6.8732 | 2 | 79.9218 | no |
| 06 | yes | yes | 7.1934 | 0 | yes | 51.4667 | 0.1301 | 7.9341 | 1 | 85.6416 | no |
| 07 | yes | yes | 13.9918 | 0 | yes | 47.0397 | 0.0873 | 8.4373 | 1 | 90.2464 | no |
| 08 | yes | yes | 35.8958 | 0 | yes | 69.0940 | 0.1929 | 9.5645 | 3 | 112.5731 | no |
| 09 | yes | yes | 49.0036 | 0 | yes | 62.9733 | 0.1743 | 1.9620 | 2 | 31.3632 | no |
| 10 | yes | yes | 34.0232 | 0 | yes | 51.1717 | 0.1535 | 2.1202 | 1 | 27.7370 | no |
| 11 | yes | yes | 8.3524 | 0 | yes | 71.4411 | 0.3277 | 5.7241 | 0 | 60.5181 | no |
| 12 | yes | yes | 45.6799 | 0 | yes | 62.0470 | 0.1659 | 3.5384 | 2 | 47.0430 | no |

[CSV](results/HyperplaneOCP/metrics.csv) · [JSON](results/HyperplaneOCP/metrics.json) · [validation](results/HyperplaneOCP/validation.json).
<!-- /results:HyperplaneOCP -->

<!-- results:VPF -->
## Virtual protection frame optimization: measured results

The paper's RK4 multiple shooting, twelve-segment frame constraints and width-enlargement loop produced 3/12 native trajectories; 3 execution optimizations succeeded and 2 attained the terminal tolerance. A fixed 40-interval mesh is used, close to the article's 35-46 intervals, after a documented development comparison with 200 intervals. Native and execution failures remain in the table. **The printed frame-cover test is not an unconditional certificate when speed reverses inside an interval.** A reproducible bounded-input unit example demonstrates this limitation; release validation also measures exact between-node motion, frame coverage and full-body collisions for actual outputs. See [the equation mapping, literal segment predicate, numerical settings and limitations](planners/VPF/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 23.0495 | — | — | — | — | — | — | — | — |
| 02 | no | no | 175.0301 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 33.8591 | 0 | yes | 25.6874 | 2.5041 | 7.1020 | 2 | 106.0615 | no |
| 04 | yes | yes | 69.1121 | 0 | no | 16.5889 | 5.3639 | 3.2660 | 1 | 91.2987 | no |
| 05 | no | no | 98.3843 | — | — | — | — | — | — | — | — |
| 06 | no | no | 209.4883 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 83.3225 | 0 | yes | 19.2899 | 2.4407 | 5.0800 | 2 | 85.2064 | no |
| 08 | no | no | 217.3814 | — | — | — | — | — | — | — | — |
| 09 | no | no | 40.9268 | — | — | — | — | — | — | — | — |
| 10 | no | no | 255.2736 | — | — | — | — | — | — | — | — |
| 11 | no | no | 143.6594 | — | — | — | — | — | — | — | — |
| 12 | no | no | 203.9460 | — | — | — | — | — | — | — | — |

[CSV](results/VPF/metrics.csv) · [JSON](results/VPF/metrics.json) · [validation](results/VPF/validation.json).
<!-- /results:VPF -->

<!-- results:RITP -->
## RITP: measured results

10/12 native solves and 10/12 evaluations succeeded; 10 executions attained the terminal tolerance, 10 had zero measured collision frames, and 10 passed both checks. Weighted polynomial path QPs and the quintic time-law QP use exact rear-axle endpoint tangents, complete sampled footprint checks, the physical flatness chain rule, common steering/rate limits and timed standstill steering at cusps. Gear phases run serially. Polynomial degrees 9 then 10 and 30 collision-weight iterations are uniform numerical choices. See [the formulation and numerical settings](planners/RITP/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 0.6525 | 0 | yes | 44.0020 | 0.5529 | 11.6737 | 2 | 132.2661 | no |
| 02 | yes | yes | 8.5913 | 0 | yes | 44.0234 | 0.8808 | 9.9976 | 3 | 123.7837 | no |
| 03 | yes | yes | 0.2128 | 0 | yes | 41.8305 | 0.3428 | 12.2108 | 1 | 130.5354 | no |
| 04 | yes | yes | 0.1244 | 0 | yes | 44.1855 | 0.3691 | 13.0764 | 1 | 139.4542 | no |
| 05 | yes | yes | 0.3312 | 0 | yes | 22.5416 | 2.0247 | 4.2389 | 1 | 67.6360 | no |
| 06 | yes | yes | 1.2338 | 0 | yes | 30.0796 | 1.8413 | 5.9919 | 1 | 83.3317 | no |
| 07 | yes | yes | 0.1021 | 0 | yes | 20.7054 | 1.4362 | 4.9712 | 1 | 69.0738 | no |
| 08 | yes | yes | 0.3255 | 0 | yes | 48.6017 | 1.2685 | 12.6245 | 3 | 153.9298 | no |
| 09 | yes | yes | 16.0473 | 0 | yes | 38.2865 | 1.4104 | 7.5040 | 1 | 94.1449 | no |
| 10 | no | no | 16.5384 | — | — | — | — | — | — | — | — |
| 11 | yes | yes | 0.4448 | 0 | yes | 29.2605 | 1.0168 | 6.8228 | 2 | 88.3952 | no |
| 12 | no | no | 7.9370 | — | — | — | — | — | — | — | — |

[CSV](results/RITP/metrics.csv) · [JSON](results/RITP/metrics.json) · [validation](results/RITP/validation.json).
<!-- /results:RITP -->

<!-- results:AnytimePSRO -->
## AnytimePSRO: measured results

12/12 native solves and 12/12 evaluations succeeded; 12 executions attained the terminal tolerance, 6 had zero measured collision frames, and 6 passed both checks. The paper's frozen-sign half-spaces, trust regions and anytime minimum-cost pool use a uniform 200-node explicit Euler grid, the common five-state rear bicycle and exact rest endpoints. Independent checks cover mesh uniformity, discrete equations, original area predicates, footprint geometry and execution. See [the formulation and numerical settings](planners/AnytimePSRO/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 3.5526 | 0 | yes | 11.8162 | 10.2691 | 2.2427 | 2 | 135.1176 | no |
| 02 | yes | yes | 2.6772 | 7.3171 | yes | 19.7191 | 3.8606 | 3.1574 | 3 | 85.1801 | no |
| 03 | yes | yes | 0.9620 | 0 | yes | 9.9852 | 10.2482 | 2.0563 | 1 | 128.0442 | no |
| 04 | yes | yes | 1.3663 | 0 | yes | 10.6313 | 12.0525 | 1.6125 | 1 | 141.6501 | no |
| 05 | yes | yes | 0.7312 | 0 | yes | 9.7041 | 10.2823 | 1.8506 | 1 | 126.3287 | no |
| 06 | yes | yes | 1.7438 | 0.2031 | yes | 10.8293 | 10.3828 | 2.0154 | 1 | 128.9817 | no |
| 07 | yes | yes | 1.4515 | 0 | yes | 9.1740 | 8.1914 | 1.8355 | 1 | 105.2681 | no |
| 08 | yes | yes | 1.7871 | 0 | yes | 15.3091 | 10.8463 | 3.4201 | 3 | 157.6639 | no |
| 09 | yes | yes | 2.1065 | 3.2546 | yes | 18.0659 | 9.1292 | 3.3458 | 3 | 139.7498 | no |
| 10 | yes | yes | 4.9270 | 2.0913 | yes | 12.3352 | 10.3740 | 1.2639 | 1 | 121.3796 | no |
| 11 | yes | yes | 0.9968 | 1.0256 | yes | 9.8461 | 5.7049 | 2.1657 | 3 | 93.7067 | no |
| 12 | yes | yes | 4.2348 | 0.4929 | yes | 17.8534 | 5.1754 | 3.0079 | 1 | 86.8336 | no |

[CSV](results/AnytimePSRO/metrics.csv) · [JSON](results/AnytimePSRO/metrics.json) · [validation](results/AnytimePSRO/validation.json).
<!-- /results:AnytimePSRO -->

<!-- results:DPGrid -->
## Backward dynamic programming on a pose grid: measured results

The backward finite-grid search produced 10/12 native paths; 10 execution optimizations succeeded and 9 attained the terminal tolerance. The primary objective minimizes arc count, with length as a tie-breaker, so a returned route can be long. Native failures remain failures. **The article permits up to 0.01 rad of heading mismatch between individually feasible arcs; this is preserved and corrected only by the independent execution evaluator.** Continuous full-body arc sweeps, independent ODE integration, angular joins, mileage spacing and exact cusp positions were checked. See [the goal-aligned fine grid, paper scope and disclosed search limits](planners/DPGrid/README.md).

Planning times include MATLAB grid construction/filtering, the single-threaded C++ search and output sampling; the one-time module build is excluded. Numerical-library threads are fixed to one. Other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 11.9542 | 0 | yes | 15.7068 | 9.6811 | 1.1625 | 2 | 118.4358 | no |
| 02 | no | no | 9.2944 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 9.7421 | 0 | yes | 22.4365 | 5.1284 | 3.2836 | 1 | 89.1201 | no |
| 04 | yes | yes | 10.1895 | 0 | yes | 20.8016 | 8.9722 | 1.8669 | 1 | 113.3919 | no |
| 05 | yes | yes | 10.7125 | 0 | yes | 55.3759 | 2.8338 | 11.8881 | 4 | 167.2191 | no |
| 06 | yes | yes | 10.7530 | 0 | yes | 37.1116 | 5.2784 | 7.4763 | 3 | 142.5472 | no |
| 07 | yes | yes | 9.9686 | 0 | yes | 26.6435 | 5.5788 | 2.7733 | 2 | 93.5211 | no |
| 08 | yes | yes | 10.8872 | 0 | yes | 44.9453 | 2.7340 | 13.0193 | 5 | 182.5328 | no |
| 09 | no | no | 8.9542 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 9.2931 | 0 | yes | 44.2227 | 4.7564 | 5.1416 | 3 | 113.9803 | no |
| 11 | yes | yes | 9.4360 | 0 | yes | 19.4739 | 14.1983 | 0.7040 | 2 | 159.0234 | no |
| 12 | yes | yes | 10.0866 | 0 | no | 38.5449 | 6.5558 | 7.1224 | 5 | 161.7818 | no |

[CSV](results/DPGrid/metrics.csv) · [JSON](results/DPGrid/metrics.json) · [validation](results/DPGrid/validation.json).
<!-- /results:DPGrid -->

<!-- results:BL_Dijkstra -->
## Barraquand-Latombe minimum-reversal search: measured results

The six-control indexed-tree Dijkstra search produced 12/12 native goal-cell paths; 12 execution optimizations succeeded and 0 attained the strict terminal tolerance. **Native success means arrival in the goal grid cell, not exact endpoint attainment.** The primary reversal-count objective can prefer a long single-direction loop; both these routes and native failures remain in the table. The original front-axle control-speed convention and the Section 6.2 pair of gear-indexing arrays are retained. Independent ODE integration, continuous swept-body checks, goal-cell membership, exact cusps and native costs were verified. See [the original algorithm, finite-resolution limitations and uniform control-duration rule](planners/BL_Dijkstra/README.md).

Planning times include setup, the single-threaded indexed search and exact path sampling. The one-time C++ build is excluded; numerical-library threads are fixed to one. Other development jobs were active, so these are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 67.7583 | 0 | no | 75.4011 | 5.5759 | 25.6792 | 4 | 332.5512 | yes |
| 02 | yes | yes | 66.5070 | 0 | no | 32.1584 | 3.6889 | 9.9411 | 4 | 156.2998 | yes |
| 03 | yes | yes | 26.8708 | 0 | no | 54.6445 | 5.8176 | 15.2645 | 1 | 215.8216 | no |
| 04 | yes | yes | 61.0103 | 0 | no | 64.2264 | 5.7928 | 16.8981 | 4 | 246.9085 | no |
| 05 | yes | yes | 39.7120 | 0 | no | 26.0449 | 2.9918 | 7.7338 | 2 | 117.2556 | yes |
| 06 | yes | yes | 41.1115 | 0 | no | 28.4264 | 2.4516 | 7.4394 | 2 | 108.9095 | no |
| 07 | yes | yes | 29.7325 | 0 | no | 58.4235 | 7.9284 | 12.6689 | 0 | 205.9733 | no |
| 08 | yes | yes | 30.6838 | 0.1842 | no | 46.1344 | 5.8303 | 11.9234 | 1 | 182.5377 | no |
| 09 | yes | yes | 57.6055 | 1.8951 | no | 72.7119 | 5.1520 | 26.4425 | 2 | 325.9450 | yes |
| 10 | yes | yes | 20.5509 | 0 | no | 44.5311 | 4.4988 | 16.4476 | 3 | 224.4636 | yes |
| 11 | yes | yes | 41.8443 | 0 | no | 24.0492 | 2.6694 | 6.9328 | 2 | 106.0222 | yes |
| 12 | yes | yes | 67.1711 | 4.2467 | no | 34.7074 | 2.9350 | 8.8416 | 3 | 132.7667 | yes |

[CSV](results/BL_Dijkstra/metrics.csv) · [JSON](results/BL_Dijkstra/metrics.json) · [validation](results/BL_Dijkstra/validation.json).
<!-- /results:BL_Dijkstra -->

<!-- results:GraphBellman -->
## Finite-element switched-system Bellman graph method: measured results

Model 1's two-mode discounted Bellman equation, simplex interpolation and selective graph updates produced 8/12 native goal-neighborhood paths; 8 execution optimizations succeeded and 0 attained the strict terminal tolerance. **Obstacles enter as the paper's soft state costs, and native termination uses a disclosed finite-grid neighborhood.** Convergence of the value function does not guarantee that the extracted continuous greedy policy reaches the goal. Failed rollouts and any collisions remain unchanged. An independent sparse MATLAB value iteration checks the C++ graph update; release checks recompute Bellman rows and policy decisions, integrate the arcs and measure full-body collisions. See [the Model 1 scope, equation mapping, target boundary and numerical choices](planners/GraphBellman/README.md).

Planning times include the entire finite-element graph construction, single-threaded Bellman solve, greedy policy extraction and output sampling. The one-time C++ build is excluded; numerical-library threads are fixed to one. Other development jobs were active, so these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 37.1569 | 0 | no | 41.3518 | 4.4937 | 5.0970 | 6 | 125.9075 | no |
| 02 | no | no | 37.5109 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 30.6033 | 0 | no | 24.9111 | 1.7836 | 5.6499 | 1 | 79.3346 | no |
| 04 | yes | yes | 54.0090 | 0 | no | 27.7289 | 2.9894 | 4.8795 | 2 | 88.6893 | no |
| 05 | no | no | 47.6115 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 35.7133 | 0 | no | 29.6296 | 2.9653 | 7.1776 | 2 | 111.4291 | no |
| 07 | yes | yes | 40.0118 | 0 | no | 44.7600 | 6.6061 | 12.1849 | 2 | 197.9110 | yes |
| 08 | yes | yes | 29.5774 | 0 | no | 42.1053 | 4.0574 | 9.3277 | 3 | 148.8509 | no |
| 09 | no | no | 38.3030 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 35.8179 | 0 | no | 38.0278 | 5.2623 | 7.9816 | 1 | 137.4389 | no |
| 11 | yes | yes | 30.5575 | 0 | no | 41.2853 | 4.9651 | 10.2362 | 3 | 167.0131 | yes |
| 12 | no | no | 33.3632 | — | — | — | — | — | — | — | — |

[CSV](results/GraphBellman/metrics.csv) · [JSON](results/GraphBellman/metrics.json) · [validation](results/GraphBellman/validation.json).
<!-- /results:GraphBellman -->

<!-- results:SmoothBiRRT -->
## Smooth-feedback bidirectional RRT*: measured results

Revised two-tree RS search, a third smoothing tree, feedback branches and a contracting sampling region produced 6/12 native paths; 6 execution optimizations succeeded and 4 attained the strict terminal tolerance. **The article's enclosing-disc model can reject a frozen task endpoint even when the physical rectangle is feasible.** Endpoint-model rejections and search-budget failures are both retained. The original length, reverse-distance, cusp and curved-length cost terms are used, with explicitly chosen unpublished weights. Independent checks recompute full path costs and terminal poses, integrate all arcs and measure dense disc/full-body clearance. See [the Algorithm 1 mapping, MATLAB time budget and disclosed smoothing/feedback choices](planners/SmoothBiRRT/README.md).

Planning times include all RS queries, tree construction, third-tree smoothing, feedback, ROI updates and path sampling. Numerical-library threads are fixed to one. Other development jobs were active; a fixed random seed with wall-time stopping does not imply bitwise repeatability across hardware or load.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 70.7639 | 0 | no | 28.9038 | 5.9189 | 9.6086 | 3 | 170.2749 | yes |
| 02 | no | no | 10.3670 | — | — | — | — | — | — | — | — |
| 03 | no | no | 9.8303 | — | — | — | — | — | — | — | — |
| 04 | yes | yes | 69.9998 | 0 | yes | 26.2516 | 2.1228 | 6.3392 | 1 | 89.6201 | no |
| 05 | no | no | 69.7556 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 70.2196 | 0 | yes | 27.3171 | 2.2589 | 7.0487 | 2 | 103.0752 | no |
| 07 | yes | yes | 70.0905 | 0 | yes | 24.0146 | 2.4027 | 7.0159 | 2 | 104.1860 | yes |
| 08 | yes | yes | 69.8689 | 0 | no | 32.3935 | 5.1368 | 12.2385 | 3 | 188.7524 | yes |
| 09 | no | no | 69.7485 | — | — | — | — | — | — | — | — |
| 10 | no | no | 9.7324 | — | — | — | — | — | — | — | — |
| 11 | yes | yes | 69.7488 | 0 | yes | 38.0751 | 4.8389 | 12.3412 | 4 | 191.8011 | yes |
| 12 | no | no | 9.6964 | — | — | — | — | — | — | — | — |

[CSV](results/SmoothBiRRT/metrics.csv) · [JSON](results/SmoothBiRRT/metrics.json) · [validation](results/SmoothBiRRT/validation.json).
<!-- /results:SmoothBiRRT -->

<!-- results:DubinsGrid -->
## Dubins lattice with Frenet-offset SQP: measured results

Forward/backward Dubins edges, a three-dimensional A* grid and scalar Frenet-offset smoothing produced 11/12 native paths. The SQP was accepted in 11 cases; 11 execution optimizations succeeded and 10 attained the strict terminal tolerance. **The paper's three width-diameter discs under-cover the physical rectangle, and the Dubins heuristic is inadmissible. Six executions collide; those same six native paths already intersect the physical obstacles while satisfying the sampled disc constraints.** Native failures and unsuccessful smoothing attempts are retained; a failed smoother returns the original search path, as allowed by the paper. Independent tests compare 186 analytic curves with MATLAB Dubins distances and ODE integration, recompute selected lattice edges, check the sampled SQP, and measure dense physical collisions separately. See [the complete paper mapping, cusp discretization and solver replacement](planners/DubinsGrid/README.md).

Planning times include grid construction, the single-threaded native A* search, Frenet-offset SQP and output conversion. The one-time C++ module build is excluded; numerical-library threads are fixed to one for SQP. Other development jobs were active, so these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 6.8086 | 0.7781 | yes | 24.1601 | 2.0432 | 6.4463 | 2 | 94.8958 | no |
| 02 | no | no | 91.6572 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 6.7591 | 0 | yes | 25.6881 | 1.9657 | 6.6546 | 1 | 91.2029 | no |
| 04 | yes | yes | 7.0481 | 0 | yes | 26.5403 | 2.1937 | 6.7787 | 2 | 99.7242 | no |
| 05 | yes | yes | 6.9503 | 0 | yes | 23.2899 | 1.9416 | 5.5614 | 2 | 85.0296 | no |
| 06 | yes | yes | 6.6697 | 0 | yes | 28.5714 | 2.3662 | 6.9798 | 2 | 103.4603 | no |
| 07 | yes | yes | 7.0134 | 0 | yes | 21.0309 | 2.0149 | 4.4276 | 2 | 74.4253 | no |
| 08 | yes | yes | 7.6214 | 2.9758 | yes | 31.2849 | 2.3344 | 8.0680 | 3 | 119.0242 | no |
| 09 | yes | yes | 8.4296 | 2.9748 | no | 26.4540 | 2.1738 | 4.7567 | 2 | 79.3055 | no |
| 10 | yes | yes | 6.4809 | 7.6667 | yes | 25.5638 | 2.5493 | 3.9657 | 2 | 75.1501 | no |
| 11 | yes | yes | 6.4091 | 5.1737 | yes | 17.2979 | 1.4540 | 5.0361 | 2 | 74.9007 | no |
| 12 | yes | yes | 6.8431 | 1.6239 | yes | 28.5714 | 2.1537 | 7.2594 | 2 | 104.1310 | no |

[CSV](results/DubinsGrid/metrics.csv) · [JSON](results/DubinsGrid/metrics.json) · [validation](results/DubinsGrid/validation.json).
<!-- /results:DubinsGrid -->

<!-- results:BicchiTangents -->
## Bicchi obstacle-supported tangent graph: measured results

Algorithm 1 with the three-circle vertex heuristic of Remark 2 produced 10/12 native paths; 10 execution optimizations succeeded and 8 attained the strict terminal tolerance. **This finite geometric graph is not complete for a rectangular car; the optional auxiliary inversion-circle search is outside the implemented scope.** Native failures remain failures. Every returned native edge passed continuous full-rectangle collision checking; release verification independently integrates the arcs, verifies supporting circles, joins, endpoint poses and exact cusps, and repeats dense and continuous body checks. Execution collisions and endpoint error are measured separately. See [the original theorem's scope, Figure 5 construction and full implementation details](planners/BicchiTangents/README.md).

Planning times include all support-circle construction, the complete directed graph and continuous collision tests, Dijkstra and exact path sampling. The one-time C++ build is excluded. Other development jobs were active, so these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 6.8209 | — | — | — | — | — | — | — | — |
| 02 | no | no | 7.1400 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 7.2029 | 0 | yes | 27.0531 | 1.5846 | 5.3108 | 2 | 78.9540 | no |
| 04 | yes | yes | 7.1874 | 0 | no | 31.8218 | 1.2144 | 7.3096 | 2 | 95.2399 | yes |
| 05 | yes | yes | 8.0171 | 0 | yes | 26.0465 | 2.1156 | 5.5278 | 2 | 86.4348 | no |
| 06 | yes | yes | 5.8493 | 0 | yes | 28.4264 | 2.3416 | 6.2654 | 2 | 96.0703 | no |
| 07 | yes | yes | 5.7355 | 0 | yes | 23.5689 | 2.0800 | 4.9758 | 3 | 85.5583 | no |
| 08 | yes | yes | 6.2501 | 0 | yes | 31.6384 | 2.7789 | 8.5741 | 3 | 128.5300 | no |
| 09 | yes | yes | 5.9357 | 0 | no | 25.6385 | 3.3216 | 3.8248 | 2 | 81.4632 | no |
| 10 | yes | yes | 5.8524 | 0 | yes | 35.2666 | 3.2140 | 6.3074 | 3 | 110.2141 | no |
| 11 | yes | yes | 5.9780 | 0 | yes | 24.2388 | 2.1259 | 5.8131 | 3 | 94.3900 | no |
| 12 | yes | yes | 5.7120 | 0 | yes | 29.6296 | 3.1990 | 8.0540 | 2 | 122.5293 | no |

[CSV](results/BicchiTangents/metrics.csv) · [JSON](results/BicchiTangents/metrics.json) · [validation](results/BicchiTangents/validation.json).
<!-- /results:BicchiTangents -->

<!-- results:TPCKC -->
## Trajectory planning with cumulative key constraints: measured results

The implicit-Euler NLP and cumulative vertex-constraint loop produced 5/12 native trajectories; 5 execution optimizations succeeded and 5 attained the strict terminal tolerance. The article's 0.04 s initial mesh, temporal propagation, objective weights and three trust radii are retained. Failed intermediate NLPs remain native failures. Independent verification checks the implicit model, objective, cumulative keys, immutable dual initialization and native full-body intersections. The analytic pose-fixed SOCP initialization agrees with an independent primal quadratic projection to 3.3e-10. **The native vertex-exclusion test can miss polygon crossings without vertex intrusion; full-body execution is measured separately.** See [the paper mapping, final-trial logic and numerical choices](planners/TPCKC/README.md).

Planning times include Hybrid A*, path-to-trajectory initialization, geometric SOCP solutions, all native NLP calls and output conversion; they differ from the paper's sum of Ipopt-only CPU times. Numerical-library threads are fixed to one. Other development jobs were active, so these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 7.4902 | 0 | yes | 11.6338 | 11.9625 | 1.7800 | 2 | 147.4253 | no |
| 02 | no | no | 47.6686 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 5.7599 | 0 | yes | 10.2762 | 9.6255 | 2.1429 | 1 | 122.6837 | no |
| 04 | no | no | 90.2620 | — | — | — | — | — | — | — | — |
| 05 | yes | yes | 7.0066 | 0 | yes | 9.7747 | 10.0726 | 1.8776 | 1 | 124.5028 | no |
| 06 | no | no | 11.2696 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 6.5942 | 0 | yes | 9.2098 | 8.1003 | 1.8409 | 1 | 104.4119 | no |
| 08 | no | no | 9.3955 | — | — | — | — | — | — | — | — |
| 09 | no | no | 28.2591 | — | — | — | — | — | — | — | — |
| 10 | no | no | 50.6740 | — | — | — | — | — | — | — | — |
| 11 | yes | yes | 36.9998 | 0 | yes | 9.3333 | 8.2070 | 1.6738 | 2 | 108.8075 | no |
| 12 | no | no | 17.9163 | — | — | — | — | — | — | — | — |

[CSV](results/TPCKC/metrics.csv) · [JSON](results/TPCKC/metrics.json) · [validation](results/TPCKC/validation.json).
<!-- /results:TPCKC -->

<!-- results:DFTPAV_Path -->
## DFTPAV static optimization, geometric path submission: measured results

The static MINCO and movable gear-shift optimization produced 11/12 native paths; 11 execution optimizations succeeded and 8 attained the strict terminal tolerance. **These measurements evaluate the optimized geometry under the common path protocol, not the article's original timed trajectory.** The native optimization retains the paper's nonzero gear-switch speed; the path adapter allows the common executor to stop at every exact cusp. Native time histories remain available as diagnostics. The complete analytic adjoint agrees with finite differences to 1.4e-7; independent checks recompute the polynomial objective, penalties, C4 joins, boundary states, arc lengths and continuous bicycle motion. Soft-penalty residuals, physical intersections and native failures are preserved. See [the paper mapping, solver replacement and explicit geometric-output scope](planners/DFTPAV_Path/README.md).

Planning times include Hybrid A*, all corridor construction, the full analytic-gradient BFGS optimization, regularity checks and arc-length output conversion. Numerical-library threads are fixed to one; these are not controlled hardware comparisons. The native polynomial duration is diagnostic only; the execution time below belongs to the common path executor.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 13.3589 | 0 | no | 28.7436 | 1.8291 | 8.6906 | 2 | 115.1973 | no |
| 02 | no | no | 64.5078 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 10.7165 | 0 | yes | 21.2121 | 1.6366 | 5.8126 | 1 | 79.4919 | no |
| 04 | yes | yes | 10.8810 | 0 | yes | 25.8065 | 1.7978 | 7.0594 | 1 | 93.5717 | no |
| 05 | yes | yes | 11.1913 | 0 | yes | 22.9508 | 1.4013 | 6.4179 | 1 | 83.1921 | no |
| 06 | yes | yes | 11.2034 | 0.7038 | yes | 23.1594 | 1.5350 | 5.1761 | 1 | 72.1111 | no |
| 07 | yes | yes | 9.6754 | 0 | yes | 23.4613 | 3.7918 | 7.3316 | 4 | 131.2335 | yes |
| 08 | yes | yes | 14.9028 | 2.5674 | no | 31.1588 | 2.8771 | 11.2693 | 5 | 166.4645 | yes |
| 09 | yes | yes | 20.4718 | 0 | yes | 32.2168 | 2.1507 | 10.3371 | 2 | 134.8785 | no |
| 10 | yes | yes | 30.6739 | 0 | yes | 32.4933 | 1.8294 | 11.3061 | 3 | 146.3555 | no |
| 11 | yes | yes | 21.2547 | 3.3004 | no | 22.5716 | 1.6185 | 5.5364 | 4 | 91.5486 | no |
| 12 | yes | yes | 16.8445 | 0 | yes | 23.1998 | 1.8430 | 4.7260 | 1 | 70.6905 | no |

[CSV](results/DFTPAV_Path/metrics.csv) · [JSON](results/DFTPAV_Path/metrics.json) · [validation](results/DFTPAV_Path/validation.json).
<!-- /results:DFTPAV_Path -->

<!-- results:HJBA -->
## Hamilton–Jacobi-guided bidirectional A*: measured results

The numerical HJ tube, exact static safe-set intersection and twenty connected-state search branches produced 12/12 native paths; 12 execution optimizations succeeded and 5 attained the strict terminal tolerance. **All reachability preparation is included in these timings, unlike the article's online-only timing.** The literal global-coordinate sampling quadrant is retained. Printed control-quantifier and heuristic-consistency issues, and all omitted numerical choices, are explicitly documented. Every successful native branch passes continuous full-rectangle collision checks; no whole-path safety guarantee is inferred from sampled reachable-set membership. Independent tests compare closed-form HJ solutions, MATLAB and compiled stencils, convex QP geometry, ODE integration and exact branch/cusp joins. See [the complete formulation, finite-grid limits and paper-to-code mapping](planners/HJBA/README.md).

Planning times include every safe-grid test, full HJ computation, pool startup/shutdown, all connected-state branches and output conversion. The compiled HJ stencil is single-threaded; up to twelve MATLAB process workers search separate connected states, each with one numerical-library thread. Its one-time build is excluded. These timings differ from the paper's online-only measurements and are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 91.3910 | 0 | no | 29.5952 | 5.3250 | 9.9481 | 3 | 167.7313 | yes |
| 02 | yes | yes | 124.1321 | 0 | no | 32.8591 | 4.6751 | 9.9613 | 4 | 166.3641 | yes |
| 03 | yes | yes | 92.2525 | 0 | yes | 27.4533 | 3.3375 | 8.4674 | 2 | 128.0491 | yes |
| 04 | yes | yes | 102.9591 | 0 | no | 29.5834 | 3.1102 | 8.2175 | 2 | 123.2772 | yes |
| 05 | yes | yes | 69.9867 | 0 | yes | 25.5991 | 3.0756 | 7.9402 | 2 | 120.1580 | yes |
| 06 | yes | yes | 76.7090 | 0 | yes | 43.1952 | 4.4254 | 9.6161 | 3 | 155.4150 | yes |
| 07 | yes | yes | 56.1067 | 0 | no | 29.3016 | 2.8514 | 8.1701 | 3 | 125.2157 | yes |
| 08 | yes | yes | 76.4677 | 2.7820 | no | 44.4996 | 3.3083 | 14.1923 | 6 | 205.0058 | yes |
| 09 | yes | yes | 60.4730 | 0 | no | 44.8000 | 3.4186 | 11.9787 | 5 | 178.9730 | no |
| 10 | yes | yes | 96.0548 | 0.9384 | yes | 49.3381 | 4.1475 | 13.6802 | 6 | 208.2771 | yes |
| 11 | yes | yes | 154.1184 | 0 | yes | 51.0683 | 4.4637 | 18.2525 | 4 | 247.1625 | yes |
| 12 | yes | yes | 63.1394 | 0 | no | 31.4640 | 4.5959 | 10.1387 | 2 | 157.3460 | yes |

[CSV](results/HJBA/metrics.csv) · [JSON](results/HJBA/metrics.json) · [validation](results/HJBA/validation.json).
<!-- /results:HJBA -->

<!-- results:BiRRT_HCR -->
## BiRRT* with HCR00-Steer: measured results

The 2018 cubic-spiral HCR00 steering function, with continuous curvature/rate between cusps, produced 7/12 native paths; 7 execution optimizations succeeded and 7 attained the terminal tolerance. The paper's five-second initial-solution deadline and three-second improvement period are retained; unsuccessful native searches remain failures. This is a distinct implementation of the HCR equations and both elementary roots, using the authors' licensed HC00 circle/tangent family geometry as its foundation. The zero-curvature endpoint variant, common-vehicle derivative limits, selected footprint-grid kernel and unpublished cost weights are explicitly documented. Independent checks cover 320 random connections, both ramp regimes and elementary constructions, sixty ODE comparisons, reversal, derivative extrema, cusp/mileage consistency and native full-body collision sampling. See [the formulation and adapter details](planners/BiRRT_HCR/README.md).

Planner time includes setup, case loading, footprint-grid construction, five-second initial search plus three-second improvement when applicable, and path conversion. In-flight edge operations can exceed a deadline slightly. The one-time MEX build is excluded. One seeded run per case is reported; these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 10.8261 | — | — | — | — | — | — | — | — |
| 02 | no | no | 10.7208 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 8.7908 | 0 | yes | 60.2151 | 2.8725 | 14.9371 | 3 | 193.0960 | no |
| 04 | yes | yes | 9.9914 | 0 | yes | 13.7790 | 12.4872 | 0.2970 | 1 | 132.8418 | no |
| 05 | no | no | 11.3295 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 8.8592 | 0 | yes | 25.1972 | 9.3701 | 2.6891 | 2 | 130.5927 | no |
| 07 | yes | yes | 8.8983 | 0 | yes | 36.9534 | 16.9963 | 3.3546 | 4 | 223.5085 | no |
| 08 | yes | yes | 9.0169 | 0 | yes | 86.1539 | 3.7315 | 16.1404 | 5 | 223.7189 | no |
| 09 | no | no | 10.8638 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 8.8213 | 0 | yes | 71.7949 | 3.3598 | 12.5413 | 4 | 179.0112 | no |
| 11 | yes | yes | 8.7478 | 0 | yes | 66.6667 | 2.7961 | 16.4823 | 3 | 207.7842 | no |
| 12 | no | no | 11.0645 | — | — | — | — | — | — | — | — |

[CSV](results/BiRRT_HCR/metrics.csv) · [JSON](results/BiRRT_HCR/metrics.json) · [validation](results/BiRRT_HCR/validation.json).
<!-- /results:BiRRT_HCR -->

<!-- results:CC_PRM -->
## CC-Steer with a directed roadmap: measured results

The 2004 continuous-curvature steering construction, embedded in a disclosed directed roadmap, produced 7/12 native paths; 7 execution optimizations succeeded and 7 attained the terminal tolerance. The connector includes both the nine printed circle families and the essential topological path; the latter was implemented separately from the licensed later CC00 source. **The roadmap is a benchmark adapter, not a reproduction of the article's PPP/ACA experiments.** Native query failures remain failures. Independent checks cover random connections, shrinking topological maneuvers, curvature continuity at all cusps, ODE integration, support-plane bounds against convex QPs, between-sample collision rejection, complete graph-route lengths and exact mileage output. See [the paper construction, source changes and finite adapter settings](planners/CC_PRM/README.md).

Planning time includes setup, case loading, the complete directed roadmap, continuous-body edge checks, Dijkstra and path conversion. The one-time native module build is excluded. One seeded call per case is reported; these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 15.7015 | — | — | — | — | — | — | — | — |
| 02 | no | no | 12.4141 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 12.4997 | 0 | yes | 27.4996 | 28.9452 | 1.1542 | 4 | 320.9940 | no |
| 04 | yes | yes | 12.8416 | 0 | yes | 14.2496 | 12.6530 | 0.3480 | 1 | 135.0100 | no |
| 05 | no | no | 12.5890 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 13.0691 | 0 | yes | 36.3767 | 34.9966 | 1.1295 | 6 | 391.2604 | no |
| 07 | yes | yes | 12.5511 | 0 | yes | 27.5813 | 24.8575 | 5.9447 | 3 | 323.0215 | no |
| 08 | yes | yes | 12.0429 | 0 | yes | 37.9246 | 38.9483 | 2.5733 | 5 | 440.2164 | no |
| 09 | no | no | 13.1416 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 12.6891 | 0 | yes | 23.0290 | 22.8562 | 1.9389 | 2 | 257.9513 | no |
| 11 | yes | yes | 12.5034 | 0 | yes | 42.7184 | 41.1792 | 5.0702 | 7 | 497.4934 | no |
| 12 | no | no | 13.0652 | — | — | — | — | — | — | — | — |

[CSV](results/CC_PRM/metrics.csv) · [JSON](results/CC_PRM/metrics.json) · [validation](results/CC_PRM/validation.json).
<!-- /results:CC_PRM -->

<!-- results:TDR_OBCA -->
## TDR_OBCA: measured results

12/12 native solves and 12/12 evaluations succeeded; 12 executions attained the terminal tolerance, 5 had zero measured collision frames, and 5 passed both checks. The temporal QP and distance-dual NLP use the common five-state rear bicycle, 200-node explicit Euler, exact task/rest endpoints and 0.01 m separation. Steering and steering rate are native state/input quantities. Native feasibility and executed collision/terminal outcomes are checked separately. See [the formulation and numerical settings](planners/TDR_OBCA/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 3.9567 | 0 | yes | 19.5838 | 9.4782 | 3.1469 | 2 | 136.2511 | no |
| 02 | yes | yes | 11.5960 | 4.4801 | yes | 20.7128 | 8.7197 | 3.5870 | 2 | 133.0669 | no |
| 03 | yes | yes | 0.7865 | 0 | yes | 14.6291 | 10.4435 | 1.2797 | 1 | 122.2318 | no |
| 04 | yes | yes | 0.6791 | 1.2780 | yes | 14.6305 | 13.3580 | 0.9393 | 1 | 147.9727 | no |
| 05 | yes | yes | 0.6614 | 0 | yes | 15.1823 | 11.2858 | 1.2979 | 1 | 130.8371 | no |
| 06 | yes | yes | 1.4116 | 0 | yes | 16.2231 | 11.0460 | 1.4740 | 1 | 130.2002 | no |
| 07 | yes | yes | 1.1228 | 0 | yes | 13.8201 | 14.7525 | 1.9275 | 1 | 171.8000 | no |
| 08 | yes | yes | 1.5926 | 1.6937 | yes | 19.8952 | 13.8166 | 3.2883 | 3 | 186.0489 | no |
| 09 | yes | yes | 4.0722 | 1.1255 | yes | 27.0977 | 15.8262 | 5.2506 | 2 | 220.7671 | no |
| 10 | yes | yes | 5.2133 | 0.5423 | yes | 18.6242 | 12.2659 | 0.6856 | 1 | 134.5156 | no |
| 11 | yes | yes | 3.4228 | 1.2152 | yes | 11.8488 | 13.1560 | 2.0380 | 1 | 156.9403 | no |
| 12 | yes | yes | 4.3179 | 0.2349 | yes | 17.8773 | 13.2372 | 2.2139 | 1 | 159.5115 | no |

[CSV](results/TDR_OBCA/metrics.csv) · [JSON](results/TDR_OBCA/metrics.json) · [validation](results/TDR_OBCA/validation.json).
<!-- /results:TDR_OBCA -->

<!-- results:RTR_TTS -->
## RTR trees with TTS/eeS continuous-curvature approximation: measured results

The paper's rotation/translation trees and complete mixed TTS/eeS approximation produced 12/12 native paths; 12 execution optimizations succeeded and 12 attained the terminal tolerance. Both stages are implemented, including TCI-interior nearest points, blocked-rotation extensions, tree intersections, sampled TTS turns, the essential exact eeS construction, reversed queries and recursive subdivision. The eeS path's curvature derivative is deliberately not bounded, as explained in the article; the common executor still enforces physical steering-rate limits. Finite query failures and execution outcomes are retained. Independent tests cover 1,021 local candidates, ODE integration, reverse traversal, shrinking maneuvers, exact translation sweeps and every successful native path's continuous full-body separation. See [the complete algorithm mapping and numerical choices](planners/RTR_TTS/README.md).

Planning time includes setup, case loading, both RT trees, all sampled/analytic local connections, recursive approximation and mileage conversion. The shared one-time MEX build is excluded. One seeded run per case is reported; these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 9.9297 | 0 | yes | 55.9901 | 20.2184 | 2.3349 | 12 | 285.5330 | no |
| 02 | yes | yes | 10.6680 | 0 | yes | 89.4215 | 16.4699 | 4.3784 | 18 | 298.4823 | no |
| 03 | yes | yes | 10.4968 | 0 | yes | 17.0637 | 11.2996 | 1.1200 | 2 | 134.1958 | no |
| 04 | yes | yes | 10.7351 | 0 | yes | 12.6395 | 11.1949 | 0.3687 | 1 | 120.6361 | no |
| 05 | yes | yes | 10.9857 | 0 | yes | 16.5548 | 6.8690 | 1.7669 | 2 | 96.3595 | no |
| 06 | yes | yes | 11.3754 | 0 | yes | 17.4959 | 11.6370 | 1.1403 | 2 | 137.7725 | no |
| 07 | yes | yes | 11.4954 | 0 | yes | 22.3347 | 15.4218 | 0.8230 | 2 | 172.4485 | no |
| 08 | yes | yes | 12.2913 | 0 | yes | 25.9339 | 12.1082 | 0.8257 | 1 | 134.3393 | no |
| 09 | yes | yes | 16.9248 | 0 | yes | 112.5700 | 29.4751 | 10.0945 | 24 | 515.6958 | no |
| 10 | yes | yes | 13.5533 | 0 | yes | 47.0452 | 14.8169 | 1.0547 | 2 | 168.7159 | no |
| 11 | yes | yes | 14.5651 | 0 | yes | 19.9922 | 15.3098 | 0.7558 | 2 | 170.6558 | no |
| 12 | yes | yes | 14.8071 | 0 | yes | 101.8885 | 17.6226 | 5.2454 | 21 | 333.6795 | no |

[CSV](results/RTR_TTS/metrics.csv) · [JSON](results/RTR_TTS/metrics.json) · [validation](results/RTR_TTS/validation.json).
<!-- /results:RTR_TTS -->

<!-- results:Sinusoid_RTR -->
## Nonlinear sinusoidal steering with RTR adapter: measured results

The nonlinear three-stage sinusoidal steering construction, combined with an explicitly separate RTR obstacle-search adapter, produced 12/12 native paths; 12 execution optimizations succeeded and 12 attained the terminal tolerance. **The 1990 article supplies a steering construction, not this complete obstacle planner; the benchmark combination is named Sinusoid_RTR to make that distinction explicit.** Exact nonlinear integrals, physical wheelbase normalization and harmonic symmetries are retained. Every native path passed continuous full-body checks, but case 6 has 0.1106% collision frames after execution. Cases 5 and 12 require approximately 728 s and 922 s of execution with 274 and 230 reversals, respectively. These long maneuvers are retained; no steering-rate or time-optimality claim is made for the native path. Independent validation covers eighty connection queries, twenty-seven near-chart-boundary cases, shrinking maneuvers, every submitted phase's physical ODE, exact cusps and continuous full-rectangle collision checks. See [the source equations, amplitude selection and obstacle adapter](planners/Sinusoid_RTR/README.md).

Planning time includes setup, case loading, RTR search, all nonlinear sinusoidal connections, quadrature, recursive approximation and mileage inversion. The one-time distance-module build is excluded. One seeded run per case is reported; these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 12.1527 | 0 | yes | 103.7097 | 15.7286 | 11.6994 | 28 | 414.2804 | no |
| 02 | yes | yes | 17.2838 | 0 | yes | 238.1713 | 38.4511 | 12.9046 | 92 | 973.5572 | no |
| 03 | yes | yes | 11.9971 | 0 | yes | 58.2963 | 5.5677 | 9.3139 | 7 | 183.8153 | no |
| 04 | yes | yes | 11.0596 | 0 | yes | 52.3684 | 6.2847 | 6.7859 | 7 | 165.7065 | no |
| 05 | yes | yes | 22.0817 | 0 | yes | 728.4463 | 71.2543 | 36.3144 | 274 | 2445.6866 | no |
| 06 | yes | yes | 10.8300 | 0.1106 | yes | 138.3262 | 9.6220 | 22.5155 | 16 | 401.3749 | no |
| 07 | yes | yes | 10.5817 | 0 | yes | 73.4828 | 6.5370 | 10.2364 | 10 | 217.7338 | no |
| 08 | yes | yes | 11.4148 | 0 | yes | 162.0044 | 14.3978 | 17.9257 | 25 | 448.2354 | no |
| 09 | yes | yes | 12.0338 | 0 | yes | 158.4920 | 16.2135 | 12.5647 | 40 | 487.7821 | no |
| 10 | yes | yes | 10.6654 | 0 | yes | 124.3588 | 10.0609 | 20.1924 | 12 | 362.5335 | no |
| 11 | yes | yes | 10.3852 | 0 | yes | 36.9679 | 4.4032 | 3.7345 | 2 | 91.3772 | no |
| 12 | yes | yes | 19.7805 | 0 | yes | 921.9265 | 92.6744 | 72.0638 | 230 | 2797.3822 | no |

[CSV](results/Sinusoid_RTR/metrics.csv) · [JSON](results/Sinusoid_RTR/metrics.json) · [validation](results/Sinusoid_RTR/validation.json).
<!-- /results:Sinusoid_RTR -->

<!-- results:KinoDeform -->
## Input-space exploration with trajectory deformation: measured results

The five-state input-shooting trees and variational trajectory deformation produced 9/12 native trajectories; 9 execution optimizations succeeded and 9 attained the terminal tolerance. The method uses no other planner for initialization or repair. Unsuccessful deformation attempts resume native exploration, and finite-budget failures remain failures. The unpublished input basis, conditioning, finite-range obstacle potential and integration settings are explicitly disclosed. Independent checks verify RK4 derivatives, the active obstacle-potential gradient, terminal sensitivities, forward/backward ODE integration, complete-body clearance, branch joins, exact zero-speed cusps and all exported fields. See [the original algorithm and numerical choices](planners/KinoDeform/README.md).

Planning time includes setup, case loading, both exploration trees, every attempted deformation, fine integration-mesh refinement and output conversion. An in-flight expansion/deformation can exceed the outer search deadline. The one-time distance-module build is excluded. One seeded run per case is reported; these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 141.8438 | 0 | yes | 30.7978 | 15.2238 | 4.5105 | 6 | 227.3436 | no |
| 02 | no | no | 222.6116 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 104.0943 | 0 | yes | 31.2593 | 11.4707 | 2.6619 | 3 | 156.3259 | no |
| 04 | no | no | 194.3913 | — | — | — | — | — | — | — | — |
| 05 | yes | yes | 52.3601 | 0 | yes | 46.6762 | 17.1377 | 4.2854 | 8 | 254.2312 | no |
| 06 | yes | yes | 51.8038 | 0 | yes | 59.6281 | 27.3787 | 6.2798 | 10 | 386.5857 | no |
| 07 | yes | yes | 124.2314 | 0 | yes | 47.6841 | 17.5943 | 5.2732 | 14 | 298.6746 | no |
| 08 | yes | yes | 36.9701 | 0 | yes | 61.7175 | 20.2781 | 9.5243 | 14 | 368.0248 | no |
| 09 | yes | yes | 140.2471 | 0 | yes | 51.7424 | 25.4054 | 6.0963 | 6 | 345.0172 | no |
| 10 | yes | yes | 43.9717 | 0 | yes | 55.4149 | 20.6998 | 5.8604 | 13 | 330.6028 | no |
| 11 | yes | yes | 25.2003 | 0 | yes | 56.0718 | 27.0636 | 6.3651 | 7 | 369.2873 | no |
| 12 | no | no | 194.1731 | — | — | — | — | — | — | — | — |

[CSV](results/KinoDeform/metrics.csv) · [JSON](results/KinoDeform/metrics.json) · [validation](results/KinoDeform/validation.json).
<!-- /results:KinoDeform -->

<!-- results:BIAGT -->
## Bidirectional improved A-search guided tree: measured results

The paper's static path-planning layer produced 8/12 native paths; 8 execution optimizations succeeded and 6 attained the terminal tolerance. Prioritized forward/reverse modes and insertion-time sharing of opposite-tree arrival costs follow the 2024 algorithm. The preceding 2019 numerical defaults, omitted tie/metric choices and exact-target RS attachment are explicitly disclosed. Dynamic-obstacle scheduling is outside these static tasks. All native successes pass full-body continuous swept-arc checks, while steering discontinuities and execution failures remain measurable. Independent verification covers 300 random RS pairs/reversals, ODE integration, shortest-length comparisons, search costs, density, shared heuristics, both mode expansions and between-sample collisions. See [the paper-to-code mapping and exact-target adapter](planners/BIAGT/README.md).

Planning time includes setup, case loading, both input-space trees, all shared-heuristic queries, continuous-body checks, terminal attachments and path conversion. The one-time native builds are excluded. One deterministic run under the published wall-time budget is reported; these wall times are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 142.3542 | 1.8335 | no | 32.9417 | 4.2881 | 10.1924 | 4 | 164.8047 | yes |
| 02 | yes | yes | 115.5168 | 6.8883 | no | 50.5333 | 9.1487 | 13.4229 | 11 | 280.7155 | yes |
| 03 | yes | yes | 14.5601 | 0.5384 | yes | 37.3333 | 3.2336 | 12.1767 | 6 | 184.1025 | no |
| 04 | yes | yes | 15.7791 | 0 | yes | 52.5913 | 10.6179 | 14.1929 | 10 | 298.1079 | yes |
| 05 | yes | yes | 14.7054 | 3.2447 | yes | 31.5264 | 4.6403 | 8.9378 | 4 | 155.7810 | yes |
| 06 | yes | yes | 27.9983 | 6.1544 | yes | 43.1549 | 4.8809 | 12.9434 | 6 | 208.2437 | yes |
| 07 | yes | yes | 14.3660 | 0 | yes | 28.8328 | 3.7904 | 8.7043 | 4 | 144.9469 | yes |
| 08 | no | no | 194.3309 | — | — | — | — | — | — | — | — |
| 09 | no | no | 194.3442 | — | — | — | — | — | — | — | — |
| 10 | no | no | 194.3317 | — | — | — | — | — | — | — | — |
| 11 | yes | yes | 15.9429 | 4.5225 | yes | 33.8953 | 3.6567 | 8.8590 | 6 | 155.1570 | yes |
| 12 | no | no | 194.6962 | — | — | — | — | — | — | — | — |

[CSV](results/BIAGT/metrics.csv) · [JSON](results/BIAGT/metrics.json) · [validation](results/BIAGT/validation.json).
<!-- /results:BIAGT -->

<!-- results:IndirectOCP -->
## Indirect optimal-control planning: measured results

The paper's planning component produced 7/12 native trajectories; 7 execution optimizations succeeded and 7 attained the terminal tolerance. Hybrid A* followed by an indirect tracking OCP initializes the complete soft-obstacle OCP on 500 nodes. The independently written canonical boundary-value solver replaces proprietary PINS; the learned execution controller is excluded. Every accepted result reaches the full obstacle objective and final regularization target. Native convergence is a first-order discrete criterion, not a minimum or continuous collision certificate. Midpoint barriers permit nodal overshoot: the largest measured steering and speed excesses were 0.0078142 rad and 0.00795622 m/s. Independent 1 ms integration of native interval controls differed from native positions by at most 0.00170569 m. These deviations and all failures are retained. The table reports the common evaluator, not that native replay. See [the exact formulation, solver replacement and numerical settings](planners/IndirectOCP/README.md).

Planning time includes setup, case loading, Hybrid A*, the tracking OCP, every successful and failed obstacle/barrier continuation attempt, and exact-cusp output conversion. Numerical-library threads are fixed to one; other development jobs were active. These are not PINS timings or controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 76.6596 | 0 | yes | 28.4264 | 1.8087 | 7.8288 | 3 | 111.3756 | no |
| 02 | no | no | 160.4380 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 28.7685 | 0 | yes | 19.5614 | 2.0655 | 3.7084 | 2 | 67.7393 | no |
| 04 | no | no | 75.0500 | — | — | — | — | — | — | — | — |
| 05 | yes | yes | 28.8096 | 0 | yes | 22.3108 | 1.6907 | 4.7264 | 2 | 74.1712 | no |
| 06 | yes | yes | 33.4944 | 0 | yes | 27.7093 | 1.8654 | 5.7647 | 2 | 86.3003 | no |
| 07 | yes | yes | 33.8929 | 0 | yes | 18.5299 | 1.5216 | 4.4445 | 2 | 69.6609 | no |
| 08 | no | no | 143.4001 | — | — | — | — | — | — | — | — |
| 09 | yes | yes | 75.7406 | 0 | yes | 37.0861 | 2.2960 | 8.5830 | 5 | 133.7901 | no |
| 10 | yes | yes | 101.7199 | 0 | yes | 24.5752 | 8.6483 | 5.4092 | 3 | 155.5756 | no |
| 11 | no | no | 94.4126 | — | — | — | — | — | — | — | — |
| 12 | no | no | 79.4657 | — | — | — | — | — | — | — | — |

[CSV](results/IndirectOCP/metrics.csv) · [JSON](results/IndirectOCP/metrics.json) · [validation](results/IndirectOCP/validation.json).
<!-- /results:IndirectOCP -->

<!-- results:PointPotentialOCP -->
## PointPotentialOCP: measured results

3/12 native solves and 3/12 evaluations succeeded; 3 executions attained the terminal tolerance, 3 had zero measured collision frames, and 3 passed both checks. The point-potential SQP uses a boundary-continuous normalized potential with the same point-exclusion feasible set, Hermite-Simpson pose collocation and linear speed/steering, with both endpoint speed and steering fixed to zero under the common limits. Each SQP call has a 60 s between-iteration budget. Native convergence, sampled mesh accuracy and common execution checks remain separate. See [the formulation and numerical settings](planners/PointPotentialOCP/README.md).

Planning time is measured through the public RunPlanner entry point, including setup, case loading, initialization, all solver attempts and result conversion. Evaluation time is excluded. One run per case and concurrent machine load do not support a controlled comparison with timings in the original paper.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 93.1217 | — | — | — | — | — | — | — | — |
| 02 | no | no | 130.0408 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 20.4625 | 0 | yes | 9.5333 | 12.1839 | 1.9791 | 1 | 146.6305 | no |
| 04 | no | no | 26.4280 | — | — | — | — | — | — | — | — |
| 05 | yes | yes | 30.1963 | 0 | yes | 9.2198 | 12.1362 | 1.7645 | 1 | 144.0070 | no |
| 06 | no | no | 25.2553 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 18.5486 | 0 | yes | 8.6661 | 10.0008 | 1.7290 | 1 | 122.2986 | no |
| 08 | no | no | 12.1607 | — | — | — | — | — | — | — | — |
| 09 | no | no | 37.2194 | — | — | — | — | — | — | — | — |
| 10 | no | no | 27.8742 | — | — | — | — | — | — | — | — |
| 11 | no | no | 52.8128 | — | — | — | — | — | — | — | — |
| 12 | no | no | 9.6801 | — | — | — | — | — | — | — | — |

[CSV](results/PointPotentialOCP/metrics.csv) · [JSON](results/PointPotentialOCP/metrics.json) · [validation](results/PointPotentialOCP/validation.json).
<!-- /results:PointPotentialOCP -->
