# CommonParking

A MATLAB benchmark for motion planning in 12 static terminal-parking scenes. The accepted scene geometry and start/goal poses are frozen. Each scene requires local maneuvering; these are not long-range driving or valet-parking route tasks.

[Forty-method result index and verification](docs/RESULT_INDEX.md) · [Evaluation protocol](evaluation/README.md) · [Result contract](docs/RESULT_FORMAT.md)

[Assessment by method category](docs/METHOD_CATEGORIES.md) · [LIOM refinement and native/executed endpoint comparison](docs/LIOM_REFINEMENT.md)

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

RITP needs MATLAB Optimization Toolbox and Parallel Computing Toolbox, plus Navigation Toolbox for Hybrid A* initialization. The planner itself has no AMPL dependency.

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
| [Point-potential optimal control](planners/PointPotentialOCP/README.md) (`PointPotentialOCP`) | Trajectory | Kondak & Hommel, ICRA 2001; direct collocation, point potentials and SQP with disclosed backend replacement | Implemented and tested on all 12 cases |
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
| [Rapid iterative trajectory planning](planners/RITP/README.md) (`RITP`) | Trajectory | Li et al., RAS 2024; polynomial QPs and parallel collision-weight iterations | Implemented and tested on all 12 cases |
| [Virtual protection frames](planners/VPF/README.md) (`VPF`) | Trajectory | Zhang et al., IET ITS 2021; RK4 multiple shooting and iterative protection frames | Implemented and tested on all 12 cases |
| [Primal hyperplane OCP](planners/HyperplaneOCP/README.md) (`HyperplaneOCP`) | Trajectory | Fan, Murgovski & Liang, TITS 2024; polytope separating planes and time/energy OCP | Implemented and tested on all 12 cases |
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
| [TriangleArea](planners/TriangleArea/README.md) (`TriangleArea`) | Trajectory | Li & Shao, KBS 2015; literal printed-model transcription | 12 cases tested; model distinction documented |

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

## H-OBCA: measured results

All twelve planner and evaluator calls succeeded. Four tracked executions satisfy the terminal tolerance; cases 2 and 8 have nonzero measured collision-frame percentages. Native collision avoidance at planner nodes does not guarantee collision-free tracked execution. The paper's steering-as-input model has no terminal zero-steering constraint; the common tracker applies its documented endpoint convention. No metric is replaced by a penalty or hidden because the result is unfavorable. No tracker time cap was active.

Wall times are measured on this desktop and include all initialization and solver stages. Other development processes were active during parts of this run; these values should not be treated as controlled hardware timing comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|
| 01 | yes | yes | 5.546 | 0.0000 | yes | 12.108 | 8.4946 | 2.1988 | 2 | 116.9343 |
| 02 | yes | yes | 137.440 | 0.0404 | no | 22.290 | 2.7229 | 4.0664 | 3 | 82.8928 |
| 03 | yes | yes | 1.573 | 0.0000 | no | 15.537 | 2.5251 | 3.8542 | 1 | 68.7933 |
| 04 | yes | yes | 1.644 | 0.0000 | no | 17.628 | 2.3138 | 3.1568 | 1 | 59.7060 |
| 05 | yes | yes | 1.615 | 0.0000 | no | 16.314 | 1.8462 | 3.7224 | 1 | 60.6860 |
| 06 | yes | yes | 3.038 | 0.0000 | yes | 12.199 | 6.6517 | 2.3078 | 1 | 94.5943 |
| 07 | yes | yes | 1.790 | 0.0000 | yes | 9.852 | 6.2427 | 2.2006 | 1 | 89.4329 |
| 08 | yes | yes | 2.269 | 0.0883 | no | 18.108 | 4.5462 | 4.5130 | 2 | 100.5918 |
| 09 | yes | yes | 2.957 | 0.0000 | no | 18.864 | 4.4634 | 4.0000 | 2 | 94.6335 |
| 10 | yes | yes | 50.873 | 0.0000 | yes | 16.566 | 3.3607 | 3.7576 | 0 | 71.1831 |
| 11 | yes | yes | 9.981 | 0.0000 | no | 25.129 | 1.9150 | 5.8597 | 1 | 82.7477 |
| 12 | yes | yes | 9.729 | 0.0000 | no | 21.039 | 2.0548 | 3.3469 | 1 | 59.0165 |

Raw data: [CSV](results/H_OBCA/metrics.csv), [JSON](results/H_OBCA/metrics.json), and standard `result` objects in `results/H_OBCA/CaseNN.mat`. [Paper correspondence](planners/H_OBCA/README.md). [RK2 and node-clearance validation](results/H_OBCA/validation.json).

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
## TriangleArea (printed model): measured results

The literal printed 2015 model solved 4/12 NLPs; all four evaluator calls succeeded, but none attained the terminal tolerance and all four had replay collisions. The independent native-model check found a maximum collocation defect of 1.04e-7, area excess at least 0.01 m² within numerical tolerance, and no collisions at native nodes in those four outputs. **The printed front-reference dynamics differ from the benchmark rear-reference bicycle model. These measurements therefore do not isolate the quality of the triangle-area collision formulation.** See [the model distinction and exact adapters](planners/TriangleArea/README.md). Failed cases are retained as failures with unavailable metrics.

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 13.2094 | — | — | — | — | — | — | — | — |
| 02 | no | no | 53.3550 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 7.2913 | 21.5131 | no | 35.2238 | 11.7909 | 10.5402 | 9 | 268.3108 | yes |
| 04 | yes | yes | 9.4767 | 1.9715 | no | 25.2072 | 12.1869 | 8.6211 | 4 | 228.0798 | yes |
| 05 | no | no | 8.2452 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 12.5239 | 32.8352 | no | 38.1438 | 17.2238 | 9.1700 | 7 | 298.9384 | yes |
| 07 | no | no | 6.6661 | — | — | — | — | — | — | — | — |
| 08 | no | no | 9.5045 | — | — | — | — | — | — | — | — |
| 09 | no | no | 8.2387 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 18.0837 | 31.3283 | no | 26.4311 | 12.7840 | 7.9377 | 4 | 227.2163 | yes |
| 11 | no | no | 5.6760 | — | — | — | — | — | — | — | — |
| 12 | no | no | 9.3042 | — | — | — | — | — | — | — | — |

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

LIOM now uses exactly three uniformly spaced covering discs and 201 states (200 intervals), with no disc-count escalation. The paper's fault-tolerant initialization, fixed 1e9 penalty and iterative corridor reconstruction are retained. An equivalent sparse objective expression and short inner solves permit useful outer iterations. 8/12 calls passed both the native solve flag and the paper infeasibility threshold; all eight executions had zero measured collision frames and attained the terminal tolerance. Cases 3 and 6 fail the fixed-cover endpoint precheck; cases 2 and 10 exhaust the outer budget. Native X/Y endpoint errors are zero in every accepted solution. The terminal column below describes the independently tracked execution, not the native hard endpoint constraints. Independent checks recomputed penalty components, objective, corridor geometry and hard bounds. See [the source mapping and settings](planners/LIOM/README.md) and [the full before/after study](docs/LIOM_REFINEMENT.md), including the rejected coarse-mesh trial and timing limitations.

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

Six cases passed the native QP/iteration criteria and independent evaluation; all six had 0% measured collision frames and attained the terminal tolerance. The other six are blocked at an endpoint by the paper's five-circle vehicle cover plus its retained 0.1 m safety margin. The implementation uses the paper's L1 linearization/feasible-set iterations and first-step perturbation, with its explicitly permitted Hybrid A* initializer. MATLAB quadprog replaces the article's CPLEX backend. Independent checks recomputed the nonlinear discrete residuals, original objective, affine and physical circle distances, motion bounds and swept footprints. See [the paper mapping and executable interpretations of the pseudocode](planners/SLiFS/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 22.3384 | 0 | yes | 15.2124 | 5.0855 | 2.1903 | 2 | 82.7580 | no |
| 02 | no | no | 1.7766 | — | — | — | — | — | — | — | — |
| 03 | no | no | 1.7381 | — | — | — | — | — | — | — | — |
| 04 | yes | yes | 5.8933 | 0 | yes | 14.3939 | 4.8198 | 0.8757 | 1 | 61.9549 | no |
| 05 | yes | yes | 3.8016 | 0 | yes | 26.5562 | 2.4348 | 3.4800 | 3 | 74.1480 | no |
| 06 | no | no | 1.7182 | — | — | — | — | — | — | — | — |
| 07 | no | no | 1.6436 | — | — | — | — | — | — | — | — |
| 08 | yes | yes | 4.5597 | 0 | yes | 16.2578 | 7.4657 | 2.9942 | 3 | 119.5986 | no |
| 09 | yes | yes | 9.0320 | 0 | yes | 29.9632 | 5.0637 | 5.6174 | 6 | 136.8114 | no |
| 10 | no | no | 1.6056 | — | — | — | — | — | — | — | — |
| 11 | yes | yes | 8.2909 | 0 | yes | 24.3149 | 8.5387 | 2.2126 | 4 | 127.5124 | no |
| 12 | no | no | 1.7198 | — | — | — | — | — | — | — | — |

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
## DL-IAPS with piecewise-jerk speed optimization: measured results

Ten calls returned trajectories and all ten passed execution optimization and attained the terminal tolerance. Case 2 exhausted the initializer's search budget; case 8 had an unsuccessful speed QP. Case 6 has a small nonzero executed collision percentage, retained in the table. The implementation follows the paper's complete quartic curvature linearization, penalty/trust-region loops and separate constant-jerk speed QPs. A uniform 20% search-curvature reserve supplies smoothing room; the optimization uses the full common vehicle limit. Independent checks recompute the discrete path constraints, full-footprint node checks, longitudinal polynomial dynamics, bounds, exact cusps and endpoint poses. These discrete constraints do not prove continuous bicycle feasibility or safety. See [the equations, endpoint-sign interpretation and differences from the Apollo source snapshot](planners/DL_IAPS_PJSO/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 4.3112 | 0 | yes | 21.8958 | 3.7957 | 4.8320 | 2 | 96.2765 | no |
| 02 | no | no | 182.8249 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 17.6811 | 0 | yes | 15.2711 | 6.4778 | 2.4219 | 1 | 93.9978 | no |
| 04 | yes | yes | 7.4755 | 0 | yes | 17.6826 | 5.2086 | 3.5917 | 1 | 93.0027 | no |
| 05 | yes | yes | 4.1344 | 0 | yes | 15.4969 | 5.8138 | 2.1961 | 1 | 85.0993 | no |
| 06 | yes | yes | 13.0686 | 0.0579 | yes | 22.4598 | 5.0240 | 3.3501 | 3 | 98.7409 | no |
| 07 | yes | yes | 17.9400 | 0 | yes | 19.9878 | 3.1578 | 3.0109 | 2 | 71.6863 | no |
| 08 | no | no | 3.5384 | — | — | — | — | — | — | — | — |
| 09 | yes | yes | 60.9611 | 0 | yes | 22.7370 | 5.7102 | 3.3701 | 3 | 105.8027 | no |
| 10 | yes | yes | 168.7406 | 0 | yes | 24.8709 | 9.9732 | 4.6281 | 2 | 156.0133 | no |
| 11 | yes | yes | 24.8487 | 0.0789 | yes | 21.5333 | 5.0647 | 3.3075 | 2 | 93.7216 | no |
| 12 | yes | yes | 28.3866 | 0 | yes | 21.1665 | 7.0589 | 3.5510 | 2 | 116.0986 | no |

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
## Timed elastic bands in distinctive topologies: measured results

The paper's sampling-based topology discovery and fixed-weight, soft-constraint LM produced 11/12 finite native outputs; 11 execution optimizations succeeded and 0 attained the terminal tolerance. **This configuration performed poorly on the terminal-parking tasks: several outputs already overlap obstacles at native nodes, and seven executions have nonzero collision percentages.** Ten static planning cycles retain the paper's four resize/optimization calls and five LM iterations per call. Native completion does not certify convergence or feasibility. These measurements concern this handwritten static adapter with the article's suggested penalty weights, not all TEB settings or the authors' complete ROS stack. The full penalty objective, positive time grid, topology exploration, selected candidate, exact output poses, LM descent history and sparse derivatives were checked independently. See [the paper equations, signed homology calculation and whole-trajectory adapter](planners/TEB/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 7.0006 | 57.3084 | no | 43.4073 | 31.9059 | 10.5404 | 33 | 589.4633 | yes |
| 02 | yes | yes | 124.0682 | 0 | no | 45.3531 | 12.9049 | 13.4895 | 20 | 363.9433 | yes |
| 03 | yes | yes | 5.8231 | 0 | no | 28.9049 | 9.7629 | 8.2922 | 9 | 225.5515 | yes |
| 04 | yes | yes | 9.3074 | 0 | no | 27.0431 | 6.4456 | 7.5374 | 4 | 159.8299 | yes |
| 05 | yes | yes | 6.5739 | 70.5149 | no | 89.5890 | 55.2758 | 17.2289 | 61 | 1030.0471 | yes |
| 06 | yes | yes | 9.9094 | 45.0499 | no | 36.5534 | 23.6713 | 9.3345 | 11 | 385.0575 | yes |
| 07 | no | no | 5.6947 | — | — | — | — | — | — | — | — |
| 08 | yes | yes | 6.0059 | 46.6086 | no | 37.9767 | 26.5128 | 8.9838 | 23 | 469.9658 | yes |
| 09 | yes | yes | 26.1409 | 46.1875 | no | 34.5298 | 20.1499 | 8.3960 | 23 | 400.4593 | yes |
| 10 | yes | yes | 44.4696 | 80.3127 | no | 62.2922 | 55.1081 | 14.5452 | 28 | 836.5323 | yes |
| 11 | yes | yes | 17.1053 | 0 | no | 33.8941 | 5.5014 | 6.9570 | 5 | 149.5844 | no |
| 12 | yes | yes | 18.6970 | 65.4293 | no | 70.6080 | 41.8616 | 18.7534 | 42 | 816.1508 | yes |

[CSV](results/TEB/metrics.csv) · [JSON](results/TEB/metrics.json) · [validation](results/TEB/validation.json).
<!-- /results:TEB -->

<!-- results:SE2_NMPC -->
## SE(2)-aware quasi-time-optimal NLP: measured results

The paper's wrapped-angle Crank-Nicolson OCP realization produced 11/12 native successes; 11 execution optimizations succeeded and 9 attained the terminal tolerance. The disclosed static adapter uses 200 intervals, the common rear-axle bicycle model, exact rectangular obstacle distances, and the article's 0.2 m safety gap. Endpoint-gap failures are retained without changing that margin. Independent checks cover native flags, all discrete dynamics, control/rate bounds, distance duals, objective, arbitrary 2*pi shifts, exact output fields and constant-input arc replay defects. See [the equation mapping and distinction from the complete feedback/ROS system](planners/SE2_NMPC/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 7.8356 | 0 | yes | 11.3913 | 12.9701 | 1.9599 | 2 | 159.2999 | no |
| 02 | yes | yes | 116.1620 | 0 | no | 34.8347 | 6.7210 | 10.8879 | 6 | 206.0891 | no |
| 03 | yes | yes | 4.6276 | 0 | yes | 10.3257 | 9.6308 | 2.1921 | 1 | 123.2287 | no |
| 04 | yes | yes | 6.8677 | 0 | yes | 10.6366 | 12.5414 | 1.6604 | 1 | 147.0185 | no |
| 05 | yes | yes | 4.9319 | 0 | yes | 10.0682 | 9.3863 | 1.9591 | 1 | 118.4546 | no |
| 06 | no | no | 3.2914 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 6.4520 | 0 | yes | 9.4558 | 7.7980 | 1.9245 | 1 | 102.2255 | no |
| 08 | yes | yes | 7.8346 | 0 | no | 38.3453 | 5.0375 | 15.0450 | 3 | 215.8256 | no |
| 09 | yes | yes | 21.4934 | 0 | yes | 23.1925 | 9.2072 | 6.6716 | 5 | 183.7876 | no |
| 10 | yes | yes | 41.9220 | 0 | yes | 16.8175 | 13.8226 | 3.9551 | 0 | 177.7768 | no |
| 11 | yes | yes | 16.9598 | 0 | yes | 17.2038 | 13.5500 | 3.2030 | 4 | 187.5305 | no |
| 12 | yes | yes | 20.6143 | 0 | yes | 17.5768 | 10.1333 | 4.2701 | 4 | 164.0338 | no |

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
## Primal separating-hyperplane optimal control: measured results

The paper's polytope separation and five-state RK4 OCP produced 6/12 native trajectories; 6 execution optimizations succeeded and all 6 attained the terminal tolerance. Cases 3 and 4 had zero measured collision frames; executions 6, 7, 10 and 11 had small nonzero collision percentages. No positive obstacle margin is added to the article's half-space constraints. The retained time/energy objective weights 1, 100 and 200 yield comparatively slow maneuvers. The other six native failures are retained. Independent checks verify the hyperplane inequalities, normal magnitudes, full objective, RK4 dynamics, endpoints, output fields and adaptive ODE replay. See [the original formulation and disclosed single-car initialization and mesh choices](planners/HyperplaneOCP/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 12.9175 | — | — | — | — | — | — | — | — |
| 02 | no | no | 9.2320 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 5.5350 | 0 | yes | 47.0475 | 0.1294 | 2.8646 | 1 | 34.9403 | no |
| 04 | yes | yes | 9.2239 | 0 | yes | 49.4143 | 0.1524 | 1.2257 | 1 | 18.7810 | no |
| 05 | no | no | 10.3893 | — | — | — | — | — | — | — | — |
| 06 | yes | yes | 25.2883 | 0.4952 | yes | 51.4894 | 0.1314 | 7.7732 | 1 | 84.0464 | no |
| 07 | yes | yes | 7.6161 | 0.6759 | yes | 46.4567 | 0.0876 | 8.2183 | 1 | 88.0587 | no |
| 08 | no | no | 13.1997 | — | — | — | — | — | — | — | — |
| 09 | no | no | 13.7464 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 24.8153 | 0.2825 | yes | 51.3303 | 0.1548 | 2.0119 | 1 | 26.6664 | no |
| 11 | yes | yes | 13.3299 | 0.5036 | yes | 66.9110 | 0.2115 | 9.3019 | 1 | 100.1337 | no |
| 12 | no | no | 65.7592 | — | — | — | — | — | — | — | — |

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
## Rapid iterative trajectory planning: measured results

The printed quintic path QP, iterative reference weights, x-only terminal alignment and quintic velocity QP produced 12/12 native trajectories; 12 execution optimizations succeeded and 5 attained the terminal tolerance. Independent gear phases run in parallel. **The printed constraints do not guarantee exact terminal tangents, curvature continuity or full polygon separation, and the reference-distance velocity mapping can disagree with the optimized position derivative.** These limitations and all native/evaluator failures remain visible. Independent checks recompute QP objectives, stationarity, time-law coefficients, output fields, cusp jumps, dense footprint intersections and physical derivative discrepancies. See [the equations and explicit differences from the later author code](planners/RITP/README.md).

Planning times include Hybrid A* and process-pool creation/shutdown. Up to four workers process independent gear phases, each with one numerical-library thread. Other development jobs were active; these are not controlled hardware comparisons or the paper's optimization-only timings.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 35.4536 | 0 | no | 46.2810 | 4.5301 | 15.9511 | 9 | 249.8120 | no |
| 02 | yes | yes | 152.9583 | 0 | no | 50.5647 | 1.5272 | 17.6283 | 5 | 216.5559 | no |
| 03 | yes | yes | 20.6998 | 0 | no | 34.0499 | 1.9141 | 10.8933 | 4 | 148.0745 | no |
| 04 | yes | yes | 19.9788 | 0 | yes | 36.2373 | 1.5756 | 12.2840 | 2 | 148.5960 | no |
| 05 | yes | yes | 19.5910 | 0 | no | 38.8889 | 3.2211 | 9.8529 | 6 | 160.7400 | no |
| 06 | yes | yes | 21.1244 | 0 | yes | 32.0000 | 1.1481 | 7.8043 | 1 | 94.5239 | no |
| 07 | yes | yes | 18.7907 | 0 | no | 33.7349 | 2.6880 | 10.2129 | 4 | 149.0086 | no |
| 08 | yes | yes | 21.3217 | 0 | no | 51.9084 | 5.2302 | 19.7952 | 11 | 305.2544 | no |
| 09 | yes | yes | 44.3541 | 0 | no | 52.8302 | 5.9903 | 19.3621 | 13 | 318.5241 | no |
| 10 | yes | yes | 59.8835 | 0 | yes | 45.3299 | 1.0864 | 15.0881 | 3 | 176.7445 | no |
| 11 | yes | yes | 34.8318 | 0 | yes | 48.0460 | 1.0886 | 11.8320 | 4 | 149.2061 | no |
| 12 | yes | yes | 71.3482 | 0 | yes | 45.5285 | 4.5350 | 11.3474 | 11 | 213.8235 | no |

[CSV](results/RITP/metrics.csv) · [JSON](results/RITP/metrics.json) · [validation](results/RITP/validation.json).
<!-- /results:RITP -->

<!-- results:AnytimePSRO -->
## Anytime predefined-space rapid optimization: measured results

Frozen-sign triangle-area half-spaces, logarithmically growing trust regions, explicit-Euler OCPs and anytime pool selection produced 10/12 native trajectories; 10 execution optimizations succeeded and 0 attained the strict terminal tolerance. Cases 2 and 9 reached the NLP iteration limit. **Six executions have nonzero collision percentages and none attains the common terminal tolerance in this configuration.** The paper's discrete model is retained; 200 nodes and explicitly chosen, otherwise unpublished objective weights are used uniformly. The full-body evaluator does not inherit the planner's vertex-exclusion approximation. Independent checks verify selected pool cost, native equations/constraints, original triangle areas, exact output timestamps and continuous-input ODE replay. See [the complete formulation, pruning envelope and disclosed numerical choices](planners/AnytimePSRO/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 22.0654 | 2.0406 | no | 16.5131 | 4.4926 | 3.3359 | 1 | 83.2852 | no |
| 02 | no | no | 135.2308 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 17.8663 | 0 | no | 12.5094 | 6.4693 | 2.6981 | 1 | 96.6744 | no |
| 04 | yes | yes | 23.6117 | 0 | no | 13.2667 | 7.3118 | 2.1738 | 1 | 99.8560 | no |
| 05 | yes | yes | 8.4365 | 0 | no | 12.1301 | 6.5122 | 2.4023 | 1 | 94.1454 | no |
| 06 | yes | yes | 21.4399 | 3.1844 | no | 14.5067 | 6.1067 | 2.8969 | 1 | 95.0367 | no |
| 07 | yes | yes | 12.5452 | 0 | no | 10.4907 | 6.6362 | 2.0658 | 1 | 92.0199 | no |
| 08 | yes | yes | 10.9139 | 3.1730 | no | 19.1913 | 6.0064 | 4.4567 | 3 | 119.6308 | no |
| 09 | no | no | 38.6320 | — | — | — | — | — | — | — | — |
| 10 | yes | yes | 46.3188 | 2.5582 | no | 19.3873 | 8.8041 | 5.5491 | 2 | 153.5317 | no |
| 11 | yes | yes | 34.4461 | 4.5146 | no | 10.8965 | 6.0814 | 2.4039 | 2 | 94.8532 | no |
| 12 | yes | yes | 20.4381 | 0.3453 | no | 14.7662 | 8.4538 | 2.5145 | 2 | 119.6833 | no |

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
## TDR-OBCA printed formulation: measured results

Temporal constant-jerk QPs, the analytically solved distance-dual QP and the printed fixed-time Euler NLP produced 12/12 native trajectories; 12 execution optimizations succeeded and 8 attained the strict terminal tolerance. **This follows the printed absolute-state and negative-distance objective, not later Apollo source variants.** Terminal pose remains soft; a disclosed hard terminal-zero-speed condition enforces the benchmark rest-to-rest task. The standalone call has no previous MPC cycle, so the prior-cycle input is zero. Unpublished weights and modified temporal-QP choices are explicit and uniform across cases. Independent tests verify the dual QP against numerical optimization, constant-jerk integration, all NLP constraints/objective, full-body node clearance and continuous-input integration. See [the equation mapping, source differences and static-task adaptations](planners/TDR_OBCA/README.md).

Planning time includes Hybrid A*, all phase speed QPs, exact distance-dual initialization, the final NLP and output conversion. Numerical-library threads are fixed to one. These measurements differ from the paper's deployed Apollo pipeline and are not controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 18.0075 | 0 | yes | 20.1525 | 8.6977 | 3.2329 | 2 | 129.3063 | no |
| 02 | yes | yes | 126.9965 | 5.9594 | yes | 32.3844 | 8.0820 | 3.9160 | 2 | 129.9797 | no |
| 03 | yes | yes | 15.7545 | 0 | yes | 14.4890 | 10.4108 | 1.3004 | 1 | 122.1123 | no |
| 04 | yes | yes | 17.1182 | 2.1611 | yes | 14.9446 | 13.2139 | 0.9071 | 1 | 146.2097 | no |
| 05 | yes | yes | 16.0026 | 0 | yes | 15.7975 | 9.7419 | 1.3714 | 1 | 116.1331 | no |
| 06 | yes | yes | 15.4161 | 0 | yes | 16.0909 | 11.0483 | 1.4982 | 1 | 130.4651 | no |
| 07 | yes | yes | 15.1623 | 0.8323 | yes | 13.8151 | 14.7484 | 1.9326 | 1 | 171.8099 | no |
| 08 | yes | yes | 23.5755 | 1.7383 | no | 21.1118 | 11.5006 | 3.3848 | 3 | 163.8541 | no |
| 09 | yes | yes | 33.8492 | 0 | no | 23.9964 | 9.1230 | 3.3475 | 2 | 134.7048 | no |
| 10 | yes | yes | 55.0737 | 0 | yes | 25.1172 | 10.8360 | 3.2955 | 0 | 141.3156 | no |
| 11 | yes | yes | 26.7340 | 0.0067 | no | 29.7624 | 8.1489 | 5.2289 | 1 | 138.7784 | no |
| 12 | yes | yes | 27.9476 | 0 | no | 25.1404 | 4.9107 | 2.9813 | 1 | 83.9197 | no |

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
## Point-potential optimal control: measured results

The direct-collocation/SQP method produced 2/12 native trajectories; 2 execution optimizations succeeded and 0 attained the terminal tolerance. The printed point-potential constraints, obstacle-free initialization and mesh-refinement loop are retained. MATLAB SQP replaces SNOPT, with disclosed interpolation, finite budgets and an optional deterministic initial-guess perturbation. Native success requires a positive solver flag and sampled mesh-accuracy checks. The printed potential has nonsmooth and potentially discontinuous branches; point exclusion is not full-polygon avoidance. Independent 1 ms integration of the native linear controls differed from native XY by at most 6.17395e-07 m. All failed initializations, obstacle solves and mesh checks remain failures; they do not establish scene infeasibility or original-SNOPT performance. See [the formula, solver replacement and explicit numerical choices](planners/PointPotentialOCP/README.md).

Planning time includes setup, case loading, obstacle-free initialization and retry when needed, every obstacle OCP and mesh-refinement attempt, mesh checks and trajectory conversion. Numerical-library threads are fixed to one; concurrent development jobs were active. These are not SNOPT timings or controlled hardware comparisons.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | no | no | 74.8212 | — | — | — | — | — | — | — | — |
| 02 | no | no | 71.5620 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 35.1346 | 0 | no | 13.6604 | 4.1211 | 3.0136 | 1 | 76.3469 | no |
| 04 | no | no | 49.1338 | — | — | — | — | — | — | — | — |
| 05 | no | no | 57.9407 | — | — | — | — | — | — | — | — |
| 06 | no | no | 72.3712 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 31.4435 | 0 | no | 10.0114 | 6.4971 | 2.0426 | 1 | 90.3974 | no |
| 08 | no | no | 103.2027 | — | — | — | — | — | — | — | — |
| 09 | no | no | 50.0144 | — | — | — | — | — | — | — | — |
| 10 | no | no | 113.1856 | — | — | — | — | — | — | — | — |
| 11 | no | no | 38.9558 | — | — | — | — | — | — | — | — |
| 12 | no | no | 44.3944 | — | — | — | — | — | — | — | — |

[CSV](results/PointPotentialOCP/metrics.csv) · [JSON](results/PointPotentialOCP/metrics.json) · [validation](results/PointPotentialOCP/validation.json).
<!-- /results:PointPotentialOCP -->
