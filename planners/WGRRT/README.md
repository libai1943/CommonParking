# Waypoint-guided two-stage RRT

Yebin Wang, Devesh K. Jha and Yukiyasu Akemi, *A two-stage RRT path planner for automated parking*, IEEE CASE 2017, pp. 496–502, DOI [10.1109/COASE.2017.8256153](https://doi.org/10.1109/COASE.2017.8256153). [MERL publication and author manuscript](https://merl.com/publications/TR2017-121).

`RunPlanner('WGRRT',caseId)` returns a **path**. The implementation follows the paper's geometric exploration, waypoint-guided bidirectional kinematic search, graph trimming, value iteration, policy extraction and path-shortening stages. It needs MATLAB Navigation Toolbox for the analytic Reeds–Shepp connections. There is no learned model, supplied solution, Hybrid A* initializer or fallback.

## Geometric exploration

Section III-B2 permits a generic geometric planner with inexpensive line steering. Here a bidirectional geometric RRT-Connect interpolates position linearly and heading along the shortest SO(2) difference. It checks the complete benchmark rectangle throughout each edge. This first-stage line interpolation need not satisfy the vehicle kinematics.

Geometric nearest-neighbor distance is translation distance plus the maximum body-point radius times the absolute heading difference. The extension step is 2 in this metric, goal bias is 0.1, and the sampling box encloses the task and obstacle vertices with a 3 m margin. Search is limited to 20,000 iterations or 40 s. Up to 150 random geometric node-pair shortcuts reduce the waypoint chain. These choices complete the geometric-planner details left open by the paper.

The geometric checker uses a 0.02 m clearance and recursively bounds the displacement of every body point. A separating-axis gap supplies a conservative distance lower bound. An unresolved interval is rejected after 18 subdivisions; a sampled-only check cannot authorize an edge. This small first-stage clearance aids numerical certification and is not a change to the benchmark obstacles.

## Waypoint-guided kinematic graph

Algorithm 5 retains the complete accumulated start graph and resets only the goal tree to the next waypoint. Samples follow the Gaussian in Section III-B3. This implementation chooses equal waypoint weights and covariance

```
mu = midpoint of the two waypoint poses (shortest wrapped heading difference)
Sigma = diag(1, 1, 0.25) * norm(nextWaypoint - mu).
```

The article leaves those positive weights unspecified. Sample headings are wrapped; samples outside the same finite task box or in body collision are rejected. The kinematic nearest-neighbor metric is Euclidean in `(x,y,wrapped_theta)`, with heading scaled by 1 m/rad. The paper's neighbor radius **6**, maximum **3** tested Reeds–Shepp candidates, and **100 new graph nodes per waypoint connection** are retained. Both trees grow toward a common sample. Successful joins can continue adding graph edges until the node threshold, as explicitly permitted in Remark 2.2. A joint sample is represented by one shared node.

The MATLAB analytic connector enumerates its Reeds–Shepp candidates. Candidates are tested in increasing length order: although one sentence says “descending,” the paper's footnote explicitly states that a candidate limit of one tests the shortest path. This implementation follows that unambiguous operational definition. Forward and reverse lengths have equal cost. Curvature is bounded by `tan(phi_max)/wheelbase`, using the common vehicle dimensions and limits.

Every connection checks the **continuous swept rectangle**. Straight motion uses the convex swept polygon; circular motion checks endpoint overlap and all corner-arc/edge contacts, including obstacle vertices against moving body edges. Tangency is collision. Only successful exact connections enter the graph. The paper does not specify this collision implementation; these geometry helpers are handwritten benchmark code.

There is a 30,000-sample guard per waypoint stage and a 180 s total kinematic-search budget. If a connection has already been found when a guard is reached, that stage's graph is retained; an unconnected stage fails explicitly. Sampling uses the reproducible seed `2017133 + caseId`, restoring the caller's RNG state afterward. This is one fixed-seed benchmark call, not the paper's 10,000-trial timing study or a probabilistic success-rate estimate.

## Graph solution and shortening

Nonterminal degree-one branches are trimmed, while the task start and goal are retained. Algorithm 6 is realized as Bellman value iteration on the undirected positive-length graph, initialized by zero at the goal and infinity elsewhere. Relaxation stops when unchanged, with at most the number of graph nodes in full sweeps. Policy recovery minimizes edge length plus the neighbor's value. Reversed edges reverse the primitive order and signed travel distances, preserving curvature as a function of signed travel. The extracted path length must agree with the start value.

The paper leaves its final `Smooth` procedure unspecified and points to sampling-planner literature. Here random mileage pairs are connected by their shortest Reeds–Shepp path; a replacement is accepted only when strictly shorter and continuously collision-free. Limits are 1,500 attempts, 150 consecutive nonimprovements or 30 s. This is path shortening, not a claim of continuous steering. It preserves the exact boundary poses and any remaining forward/reverse cusps.

## Output, limitations and validation

The final exact arc sequence is sampled uniformly within each constant-gear run with maximum spacing 0.05 m, retaining every cusp exactly. Position, body heading, front steering, mileage and interval gear are exported. The exact piecewise-circular representation accompanies the samples. No planner timestamps or physically unsupported acceleration claims are added: the paper solves its reduced three-state Reeds–Shepp model, not the five-state dynamic model discussed in its introduction. The common evaluator supplies rest-to-rest timing and separately measures executable tracking quality.

All exploration, graph construction, value iteration, shortening and output work is included in planner wall time. MATLAB uses its default numerical-library thread setting for this geometric method. Finite search failure is retained as failure and is not proof that the original scene is infeasible.

`TestWGRRT` checks shortest-candidate ordering, a thin collision barrier missed by endpoint-only checking, signed arc reversal, shortest-policy extraction from a cyclic graph with parallel edges and dead branches, and a two-stage Gaussian search. Release validation independently checks graph-edge endpoint integration and lengths, continuous collision tests, graph shortest-distance agreement, waypoint certificates, exact cusps, output spacing and adaptive quadrature of the submitted path. Native collision-free geometry and the common evaluator's execution metrics remain distinct.
