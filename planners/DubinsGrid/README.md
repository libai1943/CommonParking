# Dubins-grid search with Frenet-offset smoothing

Christoph Siedentop, Robert Heinze, Dietmar Kasper, Gabi Breuel and Cyrill Stachniss, *Path-Planning for Autonomous Parking with Dubins Curves*, Workshop Fahrerassistenzsysteme, 2015. [Author-hosted paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/siedentop15fas.pdf).

`RunPlanner('DubinsGrid',caseId)` independently implements the three-dimensional graph search of Section 4.1 and the offset-only smoothing objective and constraints of Section 4.2. It does not use Hybrid A*, Reeds–Shepp edges, an expert-generated seed path or a learned policy. Search and smoothing are separate stages, and the unsmoothed feasible search path is retained if smoothing fails, as permitted by the article's conclusion.

## Graph search

Vertices are regular world-frame XY-heading grid states, supplemented by the exact task start and goal. Each directed pair has a shortest **forward-only** Dubins curve and a shortest **backward-only** Dubins curve. The independent C++ implementation evaluates all six words (LSL, RSR, LSR, RSL, RLR, LRL) in each direction. Only the shortest word for that direction is tested against obstacles; a longer word is not substituted when it collides. A direction change is possible only at a graph vertex. With a length-only objective, the shortest collision-free edge of the two directions dominates the other.

The A* priority is accumulated length plus the smaller of the two Dubins distances to the goal. This heuristic is **inadmissible** when a shorter solution can change direction, exactly as discussed in Section 4.1.3. A state is reopened after a shorter arrival; the search stops when the goal is popped. No shortest-path guarantee is claimed for this heuristic or finite search budget. Gear is not an extra state variable, and there is no reversal-count or reverse-distance penalty: Section 5.1 explicitly identifies this limitation of the three-dimensional graph. The optional two-dimensional Dijkstra heuristic is omitted; the article reports no runtime benefit from it.

The original experiment obtains a drivable map and parking targets from previously driven trajectories. These are inputs to its planner, not a learned component of this implementation. CommonParking instead supplies its frozen polygon map and exact task poses. No prior demonstrations are required.

## Collision model and numerical choices

The original vehicle model is three discs of diameter equal to the vehicle width. Their longitudinal positions are not numerically specified; this implementation places them at the midpoints of three equal body subdivisions. **This model does not cover the rectangle's corners.** It is preserved rather than silently inflated into a different model. The independent evaluator continues to use the common physical rectangle. Point/map-boundary tests from the sensor map are replaced by exact center-to-polygon distances and an enclosing rectangular domain; touching a polygon is blocked.

| Setting | Value |
|---|---|
| World-grid XY spacing / headings | 0.75 m / 32 |
| Maximum edge length | 14 m |
| Grid/domain | Obstacles and task XY bounds plus 5 m, rounded outward to grid spacing |
| Native search budget | 180 s |
| Edge collision spacing | At most 0.05 m, including every primitive endpoint |
| Disc count / radius | 3 / half the common vehicle width |
| Smoothing nodes | About 200 total, allocated by gear-run length; at least 6 per run |
| SQP budget | 100 iterations, 60 s checked between iterations |
| Sampled constraint tolerance | 1e-5 |
| Output spacing | At most 0.05 m, uniform separately between exact cusps |

The article studies 0.25–1 m grids, 16–128 headings and 2–18 m edge limits, but does not prescribe a unique final configuration. The above choices are common to all cases. Rear-axle position must belong to the grid domain, and each tested disc must remain entirely inside its boundary. These artificial bounds are a disclosed finite-domain choice. Sampled edge tests are not continuous collision certificates.

## Frenet-offset optimization

Only scalar normal offsets from the sampled search path are optimized. The objective is the literal unweighted sum in Section 4.2.2: absolute changes of unsigned Menger curvature, absolute wrapped changes of body heading, and squared offsets. The implementation does not replace this objective with squared curvature differences or a time-minimization problem. Menger curvature uses three neighboring points; its sign, multiplied by gear, gives the steering value. Body headings use centered geometric tangents, adjusted by pi in reverse. Endpoint headings retain the original exact body poses.

The article does not specify how to discretize cusps or couple their offsets. This implementation processes each gear run separately inside a single optimization. The cusp poses remain fixed; the first two and last two offset samples in each run are zero. This preserves the parking boundary conditions and the original maneuver sequence, while allowing interior lateral motion. Curvature and disc-clearance constraints apply at sampled nodes; adjacent nodes must remain distinct. Endpoint curvature takes the nearest interior three-point value. Cost differences are taken within gear runs, not across the singular spatial tangent at a cusp.

The original solver is NLOpt SLSQP. MATLAB `fmincon` with SQP is the disclosed solver replacement. Finite differences are used, and the nonsmooth absolute-value terms remain unchanged. The paper suspects convexity, but neither convexity nor a global optimum is asserted here. A smoothed path is accepted only with a positive native solver flag, sampled constraint residual at most the stated tolerance, and a nonincreasing objective. Otherwise the original disc-checked Dubins path is returned, with the smoothing failure flag preserved in `result.solver.smoothing`.

Smoothed output is the generic sampled-path representation: piecewise-linear XY, unwrapped heading and steering, with mileage resampled uniformly within each gear run and every cusp retained. A finite Menger-curvature bound is not a continuous bicycle-model certificate. The common evaluator measures the resulting execution without inheriting the smoother's disc or discrete-curvature approximation. Unsmooth output retains an exact circular-arc descriptor.

## Build and verification

Run `SetupCommonParking` then `BuildDubinsGrid` once with a configured MATLAB C++ compiler. The source is in `native/`; the MEX file and compiler caches are built outside the repository. On Windows a portable Zig compiler can be passed as `BuildDubinsGrid('absolute/path/to/zig.exe')`. `COMMONPARKING_DUBINS_GRID_DIR` optionally selects an external build directory. The planner needs MATLAB Optimization Toolbox for SQP. Navigation Toolbox is used only by the independent analytic-curve test, not by the planner itself.

`TestDubinsGrid` compares both-direction six-word distances against MATLAB's independent Dubins implementation, integrates their bicycle equations independently, tests a known small lattice, and exhibits the three-disc corner undercoverage. Release checks retain the native search/smoothing flags and independently measure physical and disc collisions, endpoint poses, gear/cusp spacing and smoothing constraints. Compile time is excluded from online timing; grid construction, both planning stages and output conversion are included.
