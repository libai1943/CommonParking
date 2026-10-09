# Bidirectional improved A-search guided tree (BIAGT)

Yebin Wang, Emma Hansen and Heejin Ahn, *Hierarchical Planning for Autonomous Parking in Dynamic Environments*, IEEE Transactions on Control Systems Technology 32(4), 1386–1398, 2024, DOI [10.1109/TCST.2024.3367468](https://doi.org/10.1109/TCST.2024.3367468), [authors' paper](https://www.merl.com/publications/docs/TR2024-034.pdf).

This implements the paper's **static path-planning problem (Problem 3 and Section III, Algorithms 1–2)**. The dynamic-obstacle scheduling layer is outside this static benchmark and is not reproduced. BIAGT is a deterministic input-space search; it does not use a learned policy or another planner for initialization.

## Search and numerical choices

The rear-axle bicycle model is `dx/ds=gear*cos(theta)`, `dy/ds=gear*sin(theta)`, `dtheta/ds=gear*phi_n/R`. The control `phi_n` is normalized curvature, not physical wheel angle. The common vehicle gives `R=2.8/tan(0.7)`. Ten exact circular/straight primitives have length 0.175 m, five normalized steering values `[1,0.5,0,-0.5,-1]`, and two gears. The two modes are forward and reverse. These values, `delta=0.04`, `rho=1.25`, and `mu=gamma=5` come from the article's preceding ICRA 2019 reference, [Improved A-search guided tree construction for kinodynamic planning](https://www.merl.com/publications/docs/TR2019-029.pdf), DOI [10.1109/ICRA.2019.8793705](https://doi.org/10.1109/ICRA.2019.8793705). The 2024 article leaves these numerical values unspecified. Its combined BIAGT rules, rather than the earlier separate BAGT/i-AGT algorithms, determine this search.

Trees grow from both prescribed poses. Each node retains its accumulated path length, estimated remaining cost, two mode priorities and tried-mode mask. Only the highest-priority untried mode is expanded; the parent remains available until both modes have been tried. The printed binary rule gives a mode priority zero if any accepted child has smaller `F` than the parent, otherwise one. Lower is better. A fresh node inherits its parent's current priorities. Root priorities are both one. Equal priorities choose forward first; equal node keys choose the earliest inserted node. Accepted nodes are never rewired or moved.

The density test uses `sqrt(dx^2+dy^2+(R*wrap(dtheta))^2)`. This weighting, omitted in the paper, is an explicit choice. A child is rejected when any own-tree node is closer than `delta`. Before the trees approach within `mu`, they alternate. Afterwards the tree whose minimum-`F` open node has the smaller **remaining** estimate is selected; equal estimates alternate. The article says lower estimated cost-to-go without defining how to summarize a tree, so this interpretation is disclosed. An exhausted tree leaves the other active.

For each accepted node, all opposite-tree nodes within `gamma` are considered. The remaining estimate is `min(rho*RS(q,qi)+g_other(qi))`; when none is near, it is `rho*RS(q,target)`, following the 2024 article. The preceding 2019 paper instead uses the nearest opposite node for the empty-neighborhood case; that fallback is not used. Existing nodes keep their insertion-time estimate. The article notes that this inflated heuristic can yield worse paths and can be misled by the opposite tree. No optimality claim is made.

The implementation follows the printed Algorithms 1–2 target-neighborhood test. It does not add a direct inter-tree bridge operation from the brief prose mention. Reversing a goal-rooted solution reverses primitive order and signed lengths, retaining curvature.

## Exact-target adapter and collision checking

The article terminates in a ball around the target. Here the preceding experiment's 2-unit radius is retained, but a **disclosed exact-target adapter** also requires the shortest Reeds–Shepp connection to the exact target to pass the full-body collision check. A failed connection resumes tree expansion. This attachment is tried once per qualifying node; it has no alternative planner, route search or smoothing. It is not claimed to be printed in Algorithms 1–2. This addition connects the native neighborhood-based method to the benchmark's precise-pose task. Only a continuously valid attachment permits success.

The entire common vehicle rectangle is checked throughout each primitive and attachment. Conservative polygon-separation bounds and the body-motion bound `1+body_radius*abs(curvature)` certify swept clearance. Initial samples are at most 0.1 m apart; unresolved intervals are bisected down to 1e-5 m and conservatively rejected. The numerical separation guard is 1e-8 m. The workspace is the endpoint/obstacle bounding box enlarged by 3 m. Uniform finite bounds are 180 s, 100,000 mode expansions and 100,000 total nodes; an in-flight expansion can slightly exceed a bound. Failed finite searches do not establish task infeasibility.

## Output, sources and dependencies

The output is a **path**: exact prescribed endpoint poses, exact cusps, interval gears, physical wheel angle `atan(L*kappa)` and uniform mileage within each gear run at most 0.05 m apart. Steering jumps remain. The common evaluator independently measures their execution effects. Primitive controls and search diagnostics are retained; compact public results omit the large exploration trees.

Tree/mode logic, propagation, collision certification and output conversion are MATLAB. The native module provides batch shortest RS lengths and arcs. Its deterministic source comes from [steering_functions](https://github.com/hbanzhaf/steering_functions), pinned commit `79b9217697b681d953768ea0d95a9307bd312a0e`, under Apache-2.0 with the Rice/OMPL BSD attribution. Original notices and licenses are retained. The only upstream changes remove unused covariance/EKF functions and declarations; curve formulas are unchanged. Shared utilities are in the already vendored BiRRT_HC directory. CC_PRM supplies polygon-distance queries only.

```matlab
SetupCommonParking;
BuildCCSteer; % one-time shared distance module
BuildBIAGT;   % one-time RS geometry module
result = RunPlanner('BIAGT',3);
evaluation = EvaluateResult(result,LoadCase(3));
```

`BuildBIAGT('path/to/zig.exe')` supports portable Zig 0.13 on Windows instead of a configured MATLAB C++ compiler. It downloads or installs nothing. Binaries/caches stay in `tempdir/CommonParking/biagt/<architecture>`, or `COMMONPARKING_BIAGT_DIR`, outside the source tree. The planner otherwise uses base MATLAB. The independent shortest-length test additionally uses Navigation Toolbox. The evaluator has its separate AMPL/Ipopt dependency.

`TestBIAGT` checks 300 random endpoint pairs and reversals, thirty independent ODE integrations, thirty shortest-length comparisons, tree-branch integration and costs, insertion-time shared heuristics, node density, both mode expansions, and a thin obstacle between clear endpoints. Release verification independently integrates every accepted complete path at at most 0.005 m travel, checks all footprints and analytic swept arcs, and recomputes every path field and cusp. Failures and executed collisions remain in the table. Timings include the complete online search and output conversion, excluding the module build. One deterministic run under the published wall-time budget is reported per case.
