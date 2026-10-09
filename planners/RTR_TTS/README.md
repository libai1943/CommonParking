# RTR + TTS: rotation/translation trees and continuous-curvature approximation

Domokos Kiss and Gábor Tevesz, “Autonomous Path Planning for Road Vehicles in Narrow Environments: An Efficient Continuous Curvature Approach,” *Journal of Advanced Transportation*, 2017, Article 2521638, 27 pages. [Open-access article](https://doi.org/10.1155/2017/2521638).

This implementation includes **both** the article's RTR global planner and its mixed TTS/eeS local planner. It does not replace either stage with Hybrid A*, Reeds–Shepp steering, the CC-Steer family search, or a generic RRT. The paper is suitable for arbitrary static obstacle arrangements, rather than requiring parallel/perpendicular-slot metadata.

## Global stage: Algorithm 2 and Section 5

Two RT trees start at the prescribed start and goal poses. Each root first extends a straight translational configuration interval (TCI) in both forward and reverse directions until collision. A random guiding **position** is sampled; the nearest point is found over the complete tree, including the interiors of TCIs. A TCI is split when an interior point becomes a branch vertex. The metric is Euclidean position distance. Rotational intervals have constant position, so equal-distance ties use their first stored vertex; the paper does not specify tie-breaking.

An extension first rotates toward the guiding position through the smaller angular displacement, then extends straight in both directions until collision, beyond the guiding position. Even when rotation is blocked, both translations are attempted at its last free orientation; rotation in the opposite direction is then attempted from the original vertex. The two trees grow alternately. Each newly added TCI is tested for intersection with the other tree's TCIs, ordered by root depth. An intersection connects the trees only when a collision-free rotational interval joins the two orientations. Both rotation directions are tested. The final geometric path is recovered through the parent chains.

The finite sampling region is the obstacle/start/goal bounding box padded by 3 m; this artificial boundary stops otherwise unbounded translations. Rotations use 5-degree checking intervals and translations use 0.3 m intervals, with bisection to a 1e-5 m body-displacement resolution at a blocking boundary. These are **not point-only collision checks**. Body-aligned translation sweeps exactly one elongated rectangle, which is tested directly. Rotation uses full-body support-plane distance bounds and a rigid-body displacement bound to certify every interval, recursively to depth 20. Geometric-tree clearance is 0.02 m. These numerical collision choices are disclosed implementation details, not values reported by the authors.

The article's 10,000 tree-iteration limit is retained, with an additional 120 s wall-time guard. The seed is `2017065 + case_id`. Finite budgets do not preserve an unlimited-resource completeness guarantee.

## Local stage: Sections 4.3–4.6 and Appendices A–C

`Shape.m` evaluates the paper's clothoid displacement functions and the symmetric E/T-turn functions using 16-point Gauss quadrature. A factored half-angle form avoids subtracting nearly equal cosines. All configurations passed between local connections have zero curvature. Each turn consists of two symmetric clothoids, with an optional constant-curvature arc between them. The stored control rows are signed length, initial curvature, and curvature derivative with respect to **unsigned** mileage; the paper's signed sharpness therefore differs by the gear sign, but their magnitudes agree.

For each pair of configurations, `Connector.m` produces:

1. **General sampled TTS candidates.** The first E-turn deflection and signed peak curvature are sampled uniformly. The paper's EES geometry determines the second turn and the final signed straight segment. The first peak is reduced by successive factors of two until both turns meet curvature and sharpness limits, or 30 attempts are exhausted. Each E turn is then reshaped into its endpoint-equivalent T turn by the Appendix A curvature ratio, maximizing the circular fraction subject to the chosen sharpness bound. There are 32 samples per call, with no case-specific tuning. The sample count, distribution, reduction rule and root precision are unpublished numerical choices.
2. **The exact topological eeS construction.** The two E turns have opposite signed peak curvatures. The zero and maximum of `G(2*delta1,theta)` select the root/peak-curvature branch exactly as Section 4.5.2. A 65-point bracket scan plus bounded scalar refinement finds extrema, and 65 bisection steps solve bracketed roots. Equal maxima, including symmetric zero-heading ties, choose the first candidate. Coincident straight configurations use the straight-line degeneracy. No sampled TTS candidate substitutes for this essential topological construction.

Both types obey the common steering/curvature limit. The sampled TTS sharpness bound is `wmax/(wheelbase*vmax)`, a conservative choice allowing the common maximum speed. **The eeS construction has no fixed sharpness bound**, exactly as Section 4.5.4 explains: its derivative can grow as a turn shrinks, while curvature remains continuous and a sufficiently small positive traversal speed exists. We do not silently impose the sampled-TTS bound on eeS, as that would alter the topological construction. The common executor independently handles its timing and physical steering-rate limit; exact reference tracking is not guaranteed.

Candidates are sorted by total length; the first collision-free one is returned. This is equivalent to rejecting all colliding candidates and choosing the shortest survivor. If every candidate collides, start and goal are exchanged and the process is repeated; a successful connection is reversed with the exact curvature/derivative transformation. If both directions fail, the geometric path is bisected recursively. Bisection uses a pose-path parameter equal to translation distance plus rear-axle-to-corner radius times absolute heading change. Limits are 1,200 connector calls, depth 24, 180 s, and 1,000 m per local candidate. These finite numerical limits are explicit; no other planner is used on failure.

Curved local paths use a continuous full-rectangle collision certificate: the support-plane separation lower bound is compared against `(1 + body_radius*kappa_max)` times mileage displacement. The initial checking spacing is 0.15 m with every primitive endpoint included; uncertain intervals are subdivided to 1e-5 m, then conservatively rejected if unresolved. The original body dimensions are used, without a disc cover.

## Output, dependencies and validation

```matlab
SetupCommonParking;
BuildCCSteer;  % one-time shared geometry module build
result = RunPlanner('RTR_TTS',3);
evaluation = EvaluateResult(result,LoadCase(3));
```

The shared module from `CC_PRM` supplies only clothoid/arc integration and full-body support-plane distance queries; **its CC-family connector and roadmap planner are never called**. RTR trees, TTS/eeS equations, recursive approximation and all numerical choices are in this folder. Shared `ccp.Edge` and `ccp.Path` provide collision certification and standardized mileage output. The module's licensed source, attribution, portable Zig option and external binary cache are documented in [CC_PRM](../CC_PRM/README.md). This planner otherwise uses base MATLAB and needs no optimizer. The common evaluator retains its separate AMPL/Ipopt dependency.

The result is a **path**, with at most 0.05 m uniform spacing within each gear run, exact cusp samples, interval directions and front-wheel steering angles. Curvature is continuous at joins and at all cusps. The native geometric path, local controls, budgets and subdivision outcomes remain in diagnostics. Runtime includes both planning stages and output conversion; the one-time module build is excluded.

`TestRTR` checks independent random and nearly coincident connections, both turn types, endpoint accuracy, curvature continuity and bounds, reverse traversal, independent ODE integration, standardized sampling, shrinking topological maneuvers and a thin obstacle inside a swept translation. Release checks additionally integrate every returned control primitive independently, verify all geometric RTR segments and their dense body clearance, and check every native curved path continuously and at 0.005 m spacing. The benchmark execution metrics are reported independently, including collision frames and terminal errors; native success alone is not a claim of successful physical execution.
