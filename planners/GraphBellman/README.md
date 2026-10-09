# Finite-element Bellman graph algorithm

Mattia Laurini, Luca Consolini and Marco Locatelli, *A Graph-Based Algorithm for Optimal Control of Switched Systems: An Application to Car Parking*, IEEE Transactions on Automatic Control 66(12), 6049–6055, 2021. [DOI](https://doi.org/10.1109/TAC.2021.3060706) · [Author manuscript](https://aurora.ce.unipr.it/papers/laurini_consolini_locatelli_-_a_graph-based_algorithm_for_optimal_control_of_switched_systems_an_application_to_car_parking.pdf).

`RunPlanner('GraphBellman',caseId)` implements the article's **Model 1**, with two forward/backward modes, finite-element Bellman interpolation and selective graph updates. The other three automata are not claimed as additional implemented methods. No Hybrid A*, analytic endpoint connection or subsequent optimization repairs the feedback path.

## Paper formulation

The continuous pose is the rear-axle midpoint and body heading. In the forward mode its derivative is `(cos(theta),sin(theta),omega)`; in reverse it is `(-cos(theta),-sin(theta),omega)`. The unit speeds parameterize a path, not the benchmark execution velocity. The yaw-rate bound is `tan(phimax)/wheelbase`, corresponding to the article's steering relation. Five uniformly spaced yaw rates sample this interval. The current mode drives the motion for one step, and the selected automaton symbol changes the **next** mode, consistently with equation (1). The initial mode is forward, as in Model 1.

The free-state stage cost is 1; a configuration whose full physical rectangle intersects a polygon has cost `1/(1-beta)`. A direction-switch symbol adds 10. These are **soft obstacle costs**, not hard swept-body constraints. Occupied grid configurations remain in the Bellman problem. The penalty, state interpolation and finite control grid can produce a path that intersects obstacles. Such paths and between-node collisions are passed unchanged to the common evaluator. There is no undocumented collision-filtered control selector.

The objective is the discounted infinite sum in equation (3). The discrete Bellman operator is `min(stageCost + beta*interpolatedNextValue)`. It is not an undiscounted length optimum. Each Cartesian grid cell is divided into six tetrahedra by sorting its three fractional coordinates. The four barycentric coefficients define continuous, piecewise-linear finite-element interpolation. Heading is periodic; simplices crossing the heading seam use the corresponding wrapped node indices. This is simplex interpolation, not trilinear interpolation with eight vertices.

For every state/action row, the coefficient of its own value is removed and the other coefficients and constant are divided by `1-beta*alpha_self`, as in equation (8). The fixed point is computed by the largest-node-variation priority queue used for Model 1 in Section VI. The implementation follows Algorithm 3 in the authors' [graph-algorithm preprint](https://arxiv.org/abs/1809.01970), corresponding to reference 35: cache every affine row value, decrease the selected state value, update only rows depending on that state, and requeue affected states whose residual exceeds tolerance. The two-mode system is solved together, without the optional SCC decomposition used for Models 2 and 4. A separate full recomputation of the Bellman residual checks the cached updates before policy extraction.

## Disclosed numerical and task choices

The article does not provide a complete numerical specification of its mesh, discount, integration step or target boundary. This implementation uses the following settings uniformly, independently of the case identifier:

| Setting | Value |
|---|---|
| Local frame | Goal at `(0,0,0)`; rotate/translate the entire input geometry |
| Spatial grid | 0.4 m spacing, including the exact goal |
| Heading grid | 48 equally spaced headings, including the goal heading |
| Domain | Bounding rectangle of obstacles and task positions, padded by 5 m, rounded outward to grid lines |
| Motion integration | Exact constant-yaw arc, duration 0.4 at unit rear-axle speed |
| Discount | 0.98 |
| Switch penalty | 10, adopting the numerical experiment's penalty magnitude |
| Bellman residual tolerance | `1e-7` after diagonal elimination |
| Graph solve limit | 180 s, excluding graph construction and output sampling |
| Feedback rollout limit | 1000 complete motion steps |
| Output mileage spacing | At most 0.05 m in each gear run, with exact cusps |

The goal has an absorbing zero-cost boundary in both modes. On this aligned grid this fixes the value at the goal vertex. Policy rollout stops in its half-cell neighborhood: local X and Y errors at most 0.2 m and wrapped heading error at most `pi/48`. This is an explicit finite-grid boundary/termination choice, not the benchmark terminal threshold. The planner does not move the endpoint to the exact task pose. Outside the finite domain is an absorbing occupied state with value `1/(1-beta)^2`; leaving it is a policy failure. The same constant is a valid initial upper bound for all grid values because choosing no switch never costs more than the occupied-state stage cost.

After convergence, a deterministic greedy policy is evaluated at the actual continuous pose using the unmodified Bellman operator and simplex interpolation. Yaw rates are considered in ascending order, with no switch before switch to resolve exact ties. Every motion uses the current mode; a switch becomes effective on the next step. Policy extraction can fail or cycle even when the finite Bellman equation converges. Limits and partial paths are retained as failures. There is no claim of exact optimality or completeness for the original continuous problem.

## Output, dependencies and validation

The output is a path with body heading, steering, interval direction, exact cusps and a piecewise-circular descriptor. Consecutive primitives are merged only if both direction and curvature coincide. The common evaluator supplies independent rest-to-rest timing and execution. Planning wall time includes graph construction, all fixed-point updates, policy extraction and output; the one-time module compilation is excluded.

Run `SetupCommonParking; BuildGraphBellman` with a configured C++ compiler. On Windows the builder can instead accept an existing Zig 0.13 executable. The module/cache lives under `tempdir/CommonParking/graph-bellman/<arch>` or `COMMONPARKING_GRAPH_BELLMAN_DIR`, outside this repository. The planner requires base MATLAB; the public evaluator has its separate AMPL/Ipopt dependency. The geometry and graph source are independent implementations included in this folder.

`TestGraphBellman` constructs the original, untransformed sparse Bellman operators independently in MATLAB and compares synchronous value iteration with the C++ graph solution. It checks the switching convention, periodic simplex mapping and primitive lengths. Release checks independently integrate all arcs, recompute sampled Bellman rows and greedy decisions, verify goal-neighborhood membership and cusps, and measure native full-body collisions. Soft-cost collisions are measured, not silently repaired or asserted away.
