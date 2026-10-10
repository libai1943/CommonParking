# HA+CG

Reference: Dmitri Dolgov, Sebastian Thrun, Michael Montemerlo and James Diebel, *Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments*, International Journal of Robotics Research 29(5), 485–501, 2010, DOI [10.1177/0278364909359210](https://doi.org/10.1177/0278364909359210). The related 2008 short paper is *Practical Search Techniques in Path Planning for Autonomous Driving*.

## Two explicit stages

The search stage propagates continuous bicycle poses in a discretized `(x,y,theta,gear)` search space, with forward/reverse primitives and analytic Reeds-Shepp goal connections. The heuristic is the maximum of a nonholonomic obstacle-free Reeds-Shepp lower bound and a 2-D obstacle-aware Dijkstra field. Full vehicle rectangles are collision checked. This planner explicitly supplies a numerical Reeds-Shepp backend to the shared search: the heuristic computes lengths only, while analytic expansion constructs signed arc arrays directly. It avoids constructing Toolbox path objects for every candidate. MATLAB Navigation Toolbox remains a dependency for independent Dubins checks and reference tests.

The second stage uses a handwritten nonlinear conjugate-gradient solver, Polak-Ribiere+ with Armijo backtracking. It includes the paper's obstacle, curvature, smoothness and Voronoi terms, collision anchoring, and a fine refinement stage with coarse anchors retained. The implementation does not call a generic optimizer for CG.

## Reproduction choices and limits

This is a paper-based implementation, not the authors' original code. The paper leaves several numerical settings unspecified; these are explicit in `+hacg/Config.m`. Search uses 0.2 m / 5-degree cells, steering values `{-phimax,0,+phimax}`, 0.12 m clearance, gear-switch penalty 1.5 and reverse-distance penalty 0.05. Section 2.3's variable-resolution expansion uses the sum of distance to the nearest obstacle and distance to the generalized Voronoi diagram. The paper does not give the numerical mapping; this implementation chooses `step=clamp(0.1*(d_obstacle+d_GVD),0.2,1.0)` metres. It has a 100,000 expansion / 180 s search limit and tries analytic connections every five expansions.

The road/lane-graph extension is omitted because these cases provide no road network or lane graph. The search terminates at the first accepted analytic goal connection; no global shortest-path optimality is claimed. No saved search results are reused in reported planner time.

The Voronoi field uses a 0.2 m grid approximation. The curvature objective uses the paper's preceding-chord denominator; its analytic gradient is checked against finite differences both within a gear run and near gear changes. The CG objective uses the paper's terms without an added biarc penalty. The continuous output adapter uses tangent-matched circular biarcs and exact-arc checks. This representation adapter is an explicit implementation choice, not a claim that the paper specifies biarc lifting. Invalid local modifications are anchored or restored to the search path. Search success, both CG exit flags and whether the search path was retained are recorded separately.

The result is a **path**, not a timed trajectory, and does not guarantee continuous curvature or steering-rate feasibility. A retained search path is a usable planner output but must not be described as a converged CG improvement. The common evaluator supplies the cusp-to-cusp longitudinal timing and measures the resulting tracking behavior independently.

## Geometry implementation and timing

`ReedsSheppDistance.m` and `ReedsSheppCandidates.m` are MATLAB adaptations of the CSC, CCC, CCCC, CCSC and CCSCC formula families in the vendored `steering_functions` source, derived from OMPL. Time reversal, reflection and backwards configurations are included. They contain no obstacle search, saved answers or learned approximation. The [Apache license](licenses/LICENSE), [copyright notice](licenses/NOTICE) and [upstream third-party licenses](licenses/3rd-party-licenses.txt) accompany these adaptations. Distance and candidate construction are separate to avoid allocating unused paths in the heuristic.

After numerical screening finds a collision-free terminal connection, `CanonicalConnection` checks the Toolbox candidate ordering at that node and serializes its first accepted connection. This one-time normalization avoids feeding insignificant roundoff differences into sensitive CG line searches. Candidate path objects are not constructed during the preceding unsuccessful analytic expansions. The Toolbox remains required for this normalization as well as the independent validation checks.

Obstacle edge geometry and its projections are prepared once per planner call. The Boolean collision check uses the same separating axes and strict margin as the common rectangle checker, and exits as soon as one sampled pose is blocked. The distance-only boundary helper is used only for nodes that have already passed the full-body collision check; it is not an inside/outside classifier. CG skips polygon-distance calculations when both associated objective weights are zero. These changes do not reduce the search or CG budgets.

`result.computation_time_s` measures the complete public call. `result.diagnostics.timings` separately records search wall time (including preparation), the search loop, heuristic setup, CG and output conversion. Search loop and heuristic setup are components of search wall time, so these fields must not all be added together. Evaluator runtime is excluded. Public samples retain these timings, expansion counts, backend identity and both CG flags. The [stage table](../../results/HA_CG/stages.csv) makes search cost distinguishable from smoothing cost.

`TestHAGeometry` checks 3,049 pose-pair distances against Navigation Toolbox, independently replays analytic candidates to their endpoints, checks 26,400 full-body collision decisions, and compares the boundary distance at collision-free nodes. `TestCGGradient` tests both coarse and fine objectives, with and without a gear change. Every released path is also densely replayed under the full-body geometric checker. Numerical candidate enumeration can differ in tie order or floating-point details from another Reeds-Shepp implementation; no universal bitwise equivalence is claimed.

Other planners calling `parking.SearchHybridAStar(c,options)` continue to use its default backend and their existing numerical settings. They do not automatically run CG or adopt this planner's explicitly supplied geometry backend. Changing shared default geometry or a method's initialization requires its own result validation.

## Usage

```matlab
result = RunPlanner('HA_CG', 1);
caseData = LoadCase(1);
evaluation = EvaluateResult(result, caseData);
PlotEvaluation(result, evaluation, caseData);
```

Results for all twelve cases are published only after the corresponding run completes. See the repository results table for the precise evaluator version and numerical settings.
