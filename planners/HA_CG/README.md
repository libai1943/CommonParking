# HA+CG

Reference: Dmitri Dolgov, Sebastian Thrun, Michael Montemerlo and James Diebel, *Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments*, International Journal of Robotics Research 29(5), 485–501, 2010, DOI [10.1177/0278364909359210](https://doi.org/10.1177/0278364909359210). The related 2008 short paper is *Practical Search Techniques in Path Planning for Autonomous Driving*.

## Two explicit stages

The search stage propagates continuous bicycle poses in a discretized `(x,y,theta,gear)` search space, with forward/reverse primitives and analytic Reeds-Shepp goal connections. The heuristic is the maximum of a nonholonomic obstacle-free Reeds-Shepp lower bound and a 2-D obstacle-aware Dijkstra field. Full vehicle rectangles are collision checked. The implementation uses MATLAB Navigation Toolbox for Reeds-Shepp and Dubins connectors.

The second stage uses a handwritten nonlinear conjugate-gradient solver, Polak-Ribiere+ with Armijo backtracking. It includes the paper's obstacle, curvature, smoothness and Voronoi terms, collision anchoring, and a fine refinement stage with coarse anchors retained. The implementation does not call a generic optimizer for CG.

## Reproduction choices and limits

This is a paper-based implementation, not the authors' original code. The paper leaves several numerical settings unspecified; these are explicit in `+hacg/Config.m`. Search uses 0.2 m / 5-degree cells, steering values `{-phimax,0,+phimax}`, 0.12 m clearance, gear-switch penalty 1.5 and reverse-distance penalty 0.05. Section 2.3's variable-resolution expansion uses the sum of distance to the nearest obstacle and distance to the generalized Voronoi diagram. The paper does not give the numerical mapping; this implementation chooses `step=clamp(0.1*(d_obstacle+d_GVD),0.2,1.0)` metres. It has a 100,000 expansion / 180 s search limit and tries analytic connections every five expansions.

The road/lane-graph extension is omitted because these cases provide no road network or lane graph. The search terminates at the first accepted analytic goal connection; no global shortest-path optimality is claimed. No saved search results are reused in reported planner time.

The Voronoi field uses a 0.2 m grid approximation. The curvature objective uses the paper's preceding-chord denominator; its analytic gradient is checked against finite differences both within a gear run and near gear changes. The CG objective uses the paper's terms without an added biarc penalty. The continuous output adapter uses tangent-matched circular biarcs and exact-arc checks. This representation adapter is an explicit implementation choice, not a claim that the paper specifies biarc lifting. Invalid local modifications are anchored or restored to the search path. Search success, both CG exit flags and whether the search path was retained are recorded separately.

The result is a **path**, not a timed trajectory, and does not guarantee continuous curvature or steering-rate feasibility. A retained search path is a usable planner output but must not be described as a converged CG improvement. The common evaluator supplies the cusp-to-cusp longitudinal timing and measures the resulting tracking behavior independently.

## Usage

```matlab
result = RunPlanner('HA_CG', 1);
caseData = LoadCase(1);
evaluation = EvaluateResult(result, caseData);
PlotEvaluation(result, evaluation, caseData);
```

Results for all twelve cases are published only after the corresponding run completes. See the repository results table for the precise evaluator version and numerical settings.
