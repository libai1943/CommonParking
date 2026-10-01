# BiRRT* with Hybrid Curvature Steer

Holger Banzhaf, Luigi Palmieri, Dennis Nienhüser, Thomas Schamm, Steffen Knoop and J. Marius Zöllner, *Hybrid Curvature Steer: A Novel Extend Function for Sampling-Based Nonholonomic Motion Planning in Tight Environments*, ITSC 2017, DOI [10.1109/ITSC.2017.8317757](https://doi.org/10.1109/ITSC.2017.8317757).

## Method and source provenance

This implementation uses the authors' **HC±±** steering function inside a MATLAB bidirectional RRT* search. It has no Hybrid A* initializer. HC±± selects the shortest of thirteen curve families. Its clothoid/circle/line segments preserve curvature within a gear run and permit curvature jumps at gear changes. The C++ geometry is taken from [the authors' steering_functions repository](https://github.com/hbanzhaf/steering_functions), commit `79b9217697b681d953768ea0d95a9307bd312a0e`. This is a CommonParking adaptation, not the authors' ROS planner.

The vendored deterministic subset is under Apache-2.0 with the upstream BSD attribution for CC-Steer-derived code. All original file notices, `LICENSE`, `NOTICE` and `3rd-party-licenses.txt` are retained in `native/steering_functions/`. The only upstream changes remove the unused EKF/uncertainty-propagation declarations and implementations, avoiding an Eigen dependency, and add an explicit `<cassert>` include. Deterministic HC geometry and the upstream Fresnel approximation remain unchanged. The gateway, GJK implementation and MATLAB tree code are new CommonParking code.

## Tree search and cost

Two trees alternate growth from the start and the goal. A uniformly sampled free configuration is connected to a minimum-cost feasible parent in the near set (with the nearest configuration included). Local rewiring updates all descendant costs. Each inserted node is also connected to the opposite tree's near set. Feasible bridges are retained and reconsidered after rewiring. Reverse-tree controls are reversed analytically; no interpolated connector is inserted.

The paper's four cost terms are retained: path length, cusp count, normalized integral of absolute curvature, and a minimum-obstacle-clearance penalty. The last two terms are **not additive edge costs**. We aggregate length, cusp count, curvature integral, minimum clearance and boundary gear across each complete candidate branch; cross-edge cusps are counted explicitly. Descendant and complete two-tree costs are recomputed after rewiring. The best previously found feasible path is retained even if a later rewire changes a descendant's non-additive cost. No claim of a new asymptotic-optimality proof for this finite implementation is made.

Paper settings are a **6 s search budget**, 5% goal sampling, `gamma=6`, collision samples at most 0.1 m apart, 0.1 m hard obstacle clearance and an additional 0.2 m soft clearance. Every primitive endpoint is additionally checked. Broad-phase circumscribed circles precede convex GJK distance. CommonParking specifies the full rectangular body but no separate wheel outlines, so the paper's actuated-tire checks are omitted; the same full body used by the benchmark is checked. The hard margin is an exact Euclidean polygon-distance condition.

The paper does not publish the numerical cost weights or all tree implementation details. Our disclosed choices are weights `[1,2,1,1]`, a near radius `6*(log(n+1)/(n+1))^(1/3)` measured by HC path length, and uniform XY sampling in the bounding box of task endpoints and obstacle vertices enlarged by 3 m. Heading is uniform in `[-pi,pi)`. Non-root nodes have a uniformly sampled curvature sign at the maximum magnitude, so joining edges cannot introduce a same-gear curvature jump. Root curvature sign is free. The random seed is `66000+caseId`; MATLAB's caller RNG state is restored. Complete steering connections are attempted, without an additional extension-length truncation. Search ends after a complete iteration, so the measured time can slightly exceed 6 s.

## Common vehicle and output

Curvature magnitude is the common `tan(0.7)/2.8`. Sharpness is `0.5/(2.8*2.5)` per metre squared. Since `abs(phi_dot)=L*abs(sigma)*abs(v)/(1+(L*kappa)^2)`, this conservatively respects the common steering-rate bound at every curvature and at either maximum speed. These common physical limits replace the paper's different vehicle, 1 m/s speed and 10% control reserve. Steering can be nonzero at the path endpoints, as in HC±±; the evaluator independently starts and ends with zero steering.

`RunPlanner('BiRRT_HC',caseId)` returns a **path**, with exact start/goal poses, exact gear-change poses, at most 0.05 m uniform mileage spacing within each gear run, steering and interval gears. There is no fabricated timestamp. The standard evaluator applies the same piecewise-linear interpolation and cusp-to-cusp timing used for other submitted sampled paths. Native controls are retained in diagnostics for exact geometric inspection; they are not a special evaluator shortcut.

A fixed seed does not fix the number of iterations under a wall-clock budget. Published 12-case measurements are one specified run, **not** the paper's 100-repeat statistical experiment or a reproduction of its original two scenes. Planner status certifies only the implemented sampled collision checks, not continuous-time execution safety; dense execution is evaluated separately.

## One-time MATLAB build

```matlab
SetupCommonParking;
BuildHCSteer;                 % requires a C++ compiler configured with mex
result = RunPlanner('BiRRT_HC',1);
```

On Windows, `BuildHCSteer('path/to/zig.exe')` alternatively supports a portable Zig 0.13 compiler. It performs no download, purchase or installation. The tested build used Zig 0.13.0 and MATLAB R2024a. The compiled MEX and compiler caches are stored in `tempdir/CommonParking/hc-steer/<architecture>`, or the folder specified by `COMMONPARKING_HC_DIR`, outside the source tree. The compiler and MATLAB libraries are not redistributed. Planning runs in MATLAB; the native module provides batch HC connections, exact curve sampling and GJK distances.

`TestHCSteer` independently checks 1,000 random endpoint pairs, within-gear curvature continuity, curvature bounds and GJK distances against SAT plus exact edge-to-vertex geometry. Release validation additionally checks complete tree paths, reverse traversal, exact cusps and dense reference collisions. Dense reference diagnostics are separate from the common execution metrics.
