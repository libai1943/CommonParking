# Orientation-aware space exploration guided heuristic search

Chao Chen, Markus Rickert and Alois Knoll, *Path Planning with Orientation-Aware Space Exploration Guided Heuristic Search for Autonomous Parking and Maneuvering*, IEEE IV 2015, pp. 1148–1153, DOI [10.1109/IVS.2015.7225838](https://doi.org/10.1109/IVS.2015.7225838). [Authors' institute record](https://www.fortiss.org/en/research/results/scientific-publications/details/path-planning-with-orientation-aware-space-exploration-guided-heuristic-search-for-autonomous-parking-and-maneuvering).

The underlying circle construction is explained in Chen, Rickert and Knoll, *Combining Space Exploration and Heuristic Search in Online Motion Planning for Nonholonomic Vehicles*, IV 2013, DOI [10.1109/IVS.2013.6629647](https://doi.org/10.1109/IVS.2013.6629647), [author manuscript](https://archive.air.in.tum.de/Main/Publications/ChenChao2013a.pdf). The released method implements the **2015 kinematic parking planner**, with three curvature primitives and forward/reverse motions, rather than the 2013 five-state kinodynamic experiment.

`RunPlanner('OSEHS',caseId)` returns a path. MATLAB Navigation Toolbox supplies the analytic Reeds–Shepp goal connections. This planner constructs its own circle route and performs its own heuristic search; it does not use Hybrid A*, another solved scene or a learned policy.

## Directed-circle exploration

Algorithm 1 maintains open and closed sets of circles. Circles are prioritized by traveled circle distance plus remaining distance. Equation (1) defines that distance as

```
max(norm(positionA-positionB), axisAngleDifference/kappa_max).
```

The angle is reduced to [0,pi/2], as stated in the paper: a circle orientation denotes an axis, with travel direction handled separately. Actual vehicle headings and the required goal pose still distinguish a heading from its pi-shifted reverse.

Child centers are sampled uniformly on the parent circumference. Their orientation follows the connecting vector, choosing its sign to differ from the parent orientation by at most pi/2. A child is redundant when its position/orientation lies inside a previously closed circle's cylinder under equation (1). The goal connection uses the same orientation-aware distance, with the 2013 article's example overlap margin of half the smaller radius. Its candidate cost is updated until dominated by the smallest open-set cost, or a resource bound is reached. The best discovered circle chain is retained.

The 2013 construction sets radius to nearest-obstacle distance minus an inscribed vehicle radius. Here circle centers use the benchmark's rear-axle reference, so that inscribed radius is `min(rear_overhang, width/2, wheelbase+front_overhang)`. This is a **necessary geometric filter, not a complete rectangular-body collision guarantee**. The 2015 article also permits different longitudinal/lateral margins but does not prescribe their formula; this implementation keeps the explicitly defined inner-circle construction. Full-body safety is imposed by the second-stage primitive checks.

Numerical choices left unspecified in the article are 16 uniform child angles, radius range 0.02–4 m, a task/obstacle bounding box with 6 m padding, at most 120,000 generated circles and 40 s exploration time. The cap on radius bounds the exploration step in open space; a circle with less than the minimum radius is rejected. Polygon point distances are exact, not occupancy-grid approximations.

## Direction knowledge and heuristic search

The circle route is preprocessed as in Section IV. The sign of the projection toward the next circle selects preferred forward or reverse travel. If the orientation change exceeds maximum curvature times the center distance, both adjacent circles are marked bidirectional maneuvering regions.

Each vehicle state is mapped to the closest circle using equation (1); a distance tie favors the circle closer to the goal. Its heuristic is the distance to the following circle plus the remaining circle-chain distance. This is a route-guided estimate; no global optimality guarantee over other possible routes is claimed.

Section IV offers two ways to use the direction knowledge: restrict primitives, or penalize motions in the contrary direction. This implementation uses the latter. All six primitives remain available: forward/reverse at curvature `-kappa_max`, zero and `+kappa_max`. The edge length multiplier is 2 against the preferred direction and 1 otherwise. A gear change costs 1.5 m normally and 0.15 m in a bidirectional region. These penalty values are explicit implementation choices, as the paper does not give their numerical values.

The mileage step is the current circle radius times an initial factor of 0.5, with a numerical floor of 0.03 m. As in Section V, an unsuccessful search is retried with halved factors, here 0.25 and 0.125. The total search budget is 180 s, shared across the three attempts; each attempt also has a 100,000-expanded-node and 600,000-generated-node limit.

Closed vehicle states are clustered within their associated circle. Translation tolerance is `stepFactor * circleRadius`, and heading tolerance is that distance times maximum curvature. The actual wrapped heading difference is used for this equivalence test, **not the unoriented circle-axis angle**. Gear is part of the clustering state because it changes future switching cost. A lower-cost state can be expanded again. These choices realize the paper's circle-local, radius-proportional resolution without introducing a fixed global grid. This uniform setting was selected after a development probe showed excessive near-duplicate expansion with a five-times finer resolution; it is not changed per case.

When the route-based remaining distance is below 6 m, the direct goal-expansion routine enumerates analytic Reeds–Shepp connections. A collision-free candidate updates the incumbent goal cost using the same length/direction/switch penalties, applied at the start of each arc. Expansion stops when the incumbent beats the open-set priority, or the finite search limit is reached. An existing valid incumbent can be returned at that limit; native success does not certify optimality. The direct goal-connection implementation and range complete details omitted from the 2015 pseudocode.

## Exact geometry and standard output

All primitive and goal connections are checked against the complete benchmark rectangle with an analytic continuous sweep: straight convex swept polygons and circular vertex-arc/edge contacts with both moving/fixed roles. Tangency is rejected. This handwritten collision implementation does not assume that fitting a circle alone proves vehicle clearance.

The final path is an exact sequence of constant-curvature signed arcs. Samples have equal mileage spacing within each gear run, at most 0.05 m, and every cusp and both task poses are retained exactly. The standard path includes body heading, front steering, interval gear and an exact piecewise-circular geometry descriptor. Curvature jumps are permitted by the paper's kinematic model. Rest-to-rest timing and physical execution are supplied independently by the common evaluator.

There is no post-search smoothing or alternate-planner fallback. Every exploration, search and output step contributes to reported planner wall time; MATLAB uses its default numerical-library thread setting. Fixed finite bounds and the selected circle route can cause failures even when another method solves the rectangular-vehicle task.

`TestOSEHS` checks the axis metric, nearest-circle tie handling, maneuver-region marking, circle radii/expansion directions, and known optimal straight paths in both directions. Release validation additionally checks the circle-chain geometry, primitive curvature, independent integral reconstruction, exact cusps, mileage spacing and continuous swept-body collision status.
