# CommonParking

A MATLAB benchmark for motion planning in 12 static terminal-parking scenes. The accepted scene geometry and start/goal poses are frozen. Each scene requires local maneuvering; these are not long-range driving or valet-parking route tasks.

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

SLiFS additionally needs MATLAB Optimization Toolbox (`quadprog`); it has no AMPL dependency in its planner. The common evaluator still uses the external AMPL/Ipopt runtime.

Saved results can be loaded uniformly with `[result,metrics,evaluationStatus] = LoadPublishedResult('LamirauxSmooth',2)`. Most are single MAT files. Two larger artifacts are stored in lossless binary chunks because of the publication transport's request-size limit; the loader checks their byte count and SHA-256 before loading a temporary MAT file outside the repository. No samples or precision are discarded. The stored result has the same standard structure as a fresh planner call.

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
| [Smooth canonical curves](planners/LamirauxSmooth/README.md) (`LamirauxSmooth`) | Path | Lamiraux & Laumond, TRA 2001; smooth steering and holonomic-path approximation | Implemented and tested on all 12 cases |
| [Geometric subdivision + RS](planners/LaumondRS/README.md) (`LaumondRS`) | Path | Laumond et al., TRA 1994; recursive shortest-curve approximation and shortening | Implemented and tested on all 12 cases |
| [SLiFS](planners/SLiFS/README.md) (`SLiFS`) | Trajectory | Sun et al., TITS 2022; L1 convexification within circle-centre feasible sets | Implemented and tested on all 12 cases |
| [Lattice + OCP](planners/LatticeOCP/README.md) (`LatticeOCP`) | Path | Bergman et al., TIV 2021; full-model primitives and matched-cost improvement | Implemented and tested on all 12 cases |
| [LIOM](planners/LIOM/README.md) (`LIOM`) | Trajectory | Li et al., TITS 2022; fault-tolerant initialization and multi-disc extension | Implemented and tested on all 12 cases |
| [BiRRT* + HC-Steer](planners/BiRRT_HC/README.md) (`BiRRT_HC`) | Path | Banzhaf et al., ITSC 2017; authors' HC±± geometry | Implemented and tested on all 12 cases |
| [BOMP](planners/BOMP/README.md) (`BOMP`) | Trajectory | Shi et al., IJIRA 2019; 15-node pseudospectral MAKKT | Implemented and tested on all 12 cases |
| [TriangleArea](planners/TriangleArea/README.md) (`TriangleArea`) | Trajectory | Li & Shao, KBS 2015; literal printed-model transcription | 12 cases tested; model distinction documented |

Each planner has its own folder. References are named by author/title/DOI, not by a survey's numbering. Only completed implementations appear in this table. Further non-learning geometric, sampling and numerical methods are being assessed against their original papers before implementation. At most 40 methods are planned. Their original initialization and optimization methods will be respected; a shared Hybrid A* initializer is not imposed on every method. Unavailable training data and undisclosed expert rules are outside the current scope.

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

The paper's fault-tolerant initialization, fixed 1e9 penalty, 51 states (50 intervals) and iterative corridor reconstruction are retained, with the requested full-body multi-disc extension. 9/12 calls passed both the native solve flag and the paper infeasibility threshold; 9 evaluator calls succeeded and 4 executions attained the terminal tolerance. Long-duration local solutions and failures remain in the table; no global time-optimality claim is made. Independent checks recomputed every accepted penalty component, objective, corridor geometry and hard bounds. See [the complete paper mapping and numerical settings](planners/LIOM/README.md).

Planning times include initialization and optimization. Numerical-library threads are fixed to one for this method; other development jobs were active, so these wall times are not a controlled hardware comparison.

| Case | Planner | Evaluator | Plan time (s) | Collision (%) | Terminal | Execution (s) | Effort integral | Steering integral | Gear changes | Smoothness | Time cap |
|---:|:---:|:---:|---:|---:|:---:|---:|---:|---:|---:|---:|:---:|
| 01 | yes | yes | 33.9847 | 0 | yes | 123.1058 | 4.0606 | 26.5106 | 9 | 350.7120 | no |
| 02 | no | no | 1021.2213 | — | — | — | — | — | — | — | — |
| 03 | yes | yes | 26.5475 | 0 | yes | 11.4635 | 9.5801 | 1.7384 | 2 | 123.1845 | no |
| 04 | yes | yes | 7.1940 | 0 | no | 14.5930 | 7.8905 | 2.1156 | 3 | 115.0603 | no |
| 05 | yes | yes | 75.6834 | 0 | no | 12.4712 | 7.6956 | 2.2545 | 2 | 109.5008 | no |
| 06 | no | no | 768.6065 | — | — | — | — | — | — | — | — |
| 07 | yes | yes | 51.4130 | 0 | yes | 11.1934 | 7.2629 | 2.1998 | 1 | 99.6268 | no |
| 08 | yes | yes | 7.1454 | 0 | no | 31.7136 | 5.4020 | 9.0605 | 10 | 194.6253 | no |
| 09 | yes | yes | 69.8413 | 0 | yes | 32.8238 | 7.8812 | 7.8397 | 4 | 177.2083 | no |
| 10 | no | no | 1233.4159 | — | — | — | — | — | — | — | — |
| 11 | yes | yes | 92.6447 | 0 | no | 17.2039 | 10.9135 | 3.5026 | 4 | 164.1607 | no |
| 12 | yes | yes | 12.0446 | 0 | no | 50.2205 | 7.8552 | 10.7869 | 3 | 201.4210 | no |

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
