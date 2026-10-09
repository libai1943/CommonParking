# Smooth-feedback bidirectional RRT* with Reeds–Shepp curves

Jyun-Hao Jhang, Feng-Li Lian and Yu-Hsiang Hao, *Forward and Backward Motion Planning for Autonomous Parking Using Smooth-feedback Bidirectional Rapidly-exploring Random Trees* with Pattern Cost Penalty*, CASE 2020, pp. 260–265. [DOI](https://doi.org/10.1109/CASE48305.2020.9216968) · [University publication record](https://scholars.lib.ntu.edu.tw/entities/publication/9f70a247-10f4-4721-a2e1-c304be23be76).

`RunPlanner('SmoothBiRRT',caseId)` is an independent MATLAB implementation of Algorithm 1 and Sections III–IV. Its four components are the revised two-tree RS search, a third tree for smoothing, feedback of the smoothed path's nodes, and contraction of the sampling region around an improving incumbent. There is no Hybrid A* initializer or trajectory-optimization fallback. MATLAB Navigation Toolbox supplies the Reeds–Shepp curve families; the search, cost, smoothing and collision model are implemented in this folder.

## Search and smoothing

The start and goal trees grow independently. In addition to its independent sample, the goal tree attempts to reach each newly accepted start-tree node. A common pose joins the trees. Parent selection and rewiring use complete physical root-to-node costs, including the cusp at an edge join. Goal-tree paths are physically reversed when evaluating the backward-distance penalty; reversing only their sample order would give the wrong cost. Rewiring updates the cost of every descendant and excludes ancestors to avoid cycles.

Each candidate edge is selected from the finite, analytic Reeds–Shepp families by the weighted cost below. Its arc length must be at most the 10 m rewire radius. The least-cost family is tested for collision; another family is not substituted when that edge collides. All existing nodes within the Euclidean radius are considered as potential parents, with the RS length check providing the second filter. This whole-sample insertion follows the RRT* structure cited by the article. The referenced [Henderson repository](https://github.com/sportdeath/motion_planning/tree/c4cc966b8c0fb2f12cf61272584756244766d96a) was inspected for its insertion and rewiring conventions; its source is not bundled or translated.

A connected path is interpolated at up to 0.5 m spacing, retaining every exact primitive boundary. A new tree rooted at the task start processes these poses in path order, choosing parents and rewiring as in RRT*. The known original subarc is retained as a feasible parent option, so smoothing can return its incumbent if the time budget expires. The resulting path is accepted only if its complete cost is no larger than the connected path. Its interpolated nodes and subarcs are then fed back as complete branches into **both** original trees, including the physical reverse for the goal tree. This feedback is performed for each connection, even when it does not improve the global incumbent.

For an improving incumbent, the XY sampling rectangle becomes the bounding box of its sampled path, padded by 2 m and intersected with the initial domain. Heading sampling retains the full circle. The article describes expansion of the solution for this region update but does not specify a numerical margin. Existing nodes outside the new region are retained.

The article does not give detailed code for `Optimize`, `SmoothFeedback`, or all numerical settings. The above third-tree parent option, ordered processing, path-node feedback and 2 m region rule are explicit realization choices. They are not claimed to reproduce the authors' exact random trees. Because a cusp cost depends on incoming direction, a single pose representative and finite rewiring do not establish a global or asymptotic optimality certificate.

## Vehicle and collision model

The benchmark vehicle dimensions and minimum turning radius replace the experiment's minivan and 5 m turning radius. The article's equation (2) has cosine/sine increments inconsistent with the stated rear-axle heading convention. This implementation follows the Reeds–Shepp geometry and bicycle steering relation in equation (1): exact circular arcs and straight segments in the standard rear-axle frame.

Equations (3)–(4) define equally spaced enclosing circles. Their radius here is `width/2 + 0.19 m`; the 0.19 m is the side-mirror addition in Figure 6, with zero additional buffer. The benchmark body itself is unchanged: this extra width is the planner's conservative margin. The smallest integer circle count satisfying `sqrt((length/(2*count))^2 + (width/2)^2) <= radius` is used. Centers are at the midpoint of each longitudinal body subdivision, measured from the rear axle. A circle intersects a polygon if its center is inside or its distance to an edge is at most the radius.

Collision tests sample an edge at mileage spacing at most 0.05 m, including each arc endpoint. These are the paper's sampled disc tests, not continuous swept-body certificates. A required endpoint blocked by the enclosing-disc model is returned as a native failure. Neither the circle radius nor the frozen case is adjusted to force a success. The evaluator independently uses the physical rectangle at its own execution frames.

## Objective and numerical choices

Equations (5)–(9) penalize total length, reverse length, cusp count and curved-segment length. The last term is arc length on nonzero-curvature RS pieces, not squared curvature. Cusps between different tree edges count too. The paper leaves the four penalty coefficients symbolic; this implementation uses `[1,1,5,1]` for these terms. This internal search objective is separate from the public multi-dimensional evaluation and is never used as an overall benchmark score.

| Parameter | Setting |
|---|---|
| Search budget | 60 s, at most 20,000 outer iterations |
| Rewire radius | 10 m RS length, with an initial Euclidean filter |
| Initial domain | Obstacles and task positions padded by 5 m |
| Uniform heading samples | `[-pi,pi)` |
| Root-directed sample probability | 0.05 |
| Interpolation for the third tree | 0.5 m plus exact arc boundaries |
| Collision samples / output samples | At most 0.05 m |
| Seed | `2020105 + caseId`, with the caller's RNG state restored |

The original C++ experiment uses a 3 s budget. The disclosed 60 s budget accommodates this MATLAB implementation and is applied to every case. A run finishes the current bounded operation and preserves a valid incumbent; it may slightly exceed the budget. All setup, interpolation, feedback, search and output work is included in `RunPlanner` wall time. Wall-time stopping means trajectories need not be bitwise identical on different hardware or under different loads, even with a fixed seed.

The standard output is a path with exact analytic arcs, body heading, steering, interval gears and exact cusps. Uniform mileage samples are produced separately within each gear run. `TestSmoothBiRRT` checks the complete cost including inter-edge cusps and physical reversal, circle containment/contact and a free-space query that exercises the third tree and feedback. Release validation independently integrates the arcs, recomputes objective terms and terminal pose, verifies cusp spacing, and measures dense circle/full-body clearance without repairing the result.
