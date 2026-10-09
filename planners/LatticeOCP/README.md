# Lattice planning with matched optimal control

Kristoffer Bergman, Oskar Ljungqvist and Daniel Axehill, *Improved Path Planning by Tightly Combining Lattice-Based Path Planning and Optimal Control*, IEEE Transactions on Intelligent Vehicles 6(1), 57–66, 2021, DOI [10.1109/TIV.2020.2991951](https://doi.org/10.1109/TIV.2020.2991951).

This implementation follows the **complete car model** and the matched objective in all three stages: offline primitive optimization, A* graph search, and online multiphase optimal control. It does not replace the lattice initializer with Hybrid A*. It is an independent MATLAB/AMPL implementation with native MATLAB graph routines, not the authors' C++/Python distribution or a reproduction of their original experiments.

## Continuous model and objective

The independent variable is travelled distance `s`. The state is `[x,y,theta,alpha,omega]`, where `alpha` is steering and **omega means d(alpha)/ds**, not steering rate per second. The control `u` is d(omega)/ds. The mode `q=+1/-1` is the driving direction:

```
x' = q*cos(theta), y' = q*sin(theta), theta' = q*tan(alpha)/L
alpha' = omega, omega' = u
J = integral(1 + alpha^2 + 10*omega^2 + u^2) ds
```

This is Example 1 and Eq. (14), with the paper's `gamma=1`. CommonParking replaces the paper's vehicle dimensions and steering limit with the common vehicle. The spatial steering-rate bound is `0.5/2.5 = 0.2 rad/m`, so the common 0.5 rad/s limit is respected up to either maximum speed. The paper's spatial steering-acceleration bound 40 is retained; CommonParking specifies no corresponding time-domain steering-acceleration limit.

## Offline preparation

The state lattice uses a 1 m position grid, sixteen irregular headings whose directions are the primitive integer vectors `(1,0),(2,1),(1,1),(1,2)` and their quarter rotations. Steering and its spatial derivative are zero at lattice vertices. Table I's complete-model set has four neighboring heading changes in each direction, three lateral shifts in each direction, and one straight primitive, all in forward and reverse motion: **480 primitives**.

Primitive generation follows the maneuver-based endpoint optimization described in the paper's reference to Bergman et al., *Improved Optimization of Motion Primitives for Motion Planning in State Lattices*, IV 2019, [arXiv:1810.07470](https://arxiv.org/abs/1810.07470). Heading-change maneuvers first optimize free XY endpoints. Parallel maneuvers first constrain the local lateral offset while leaving longitudinal progress free. Nearby grid endpoints are then reoptimized, and the lowest-cost feasible result is retained. The implementation considers sixteen nearby XY alternatives for heading changes. For lateral shifts it rounds the requested lateral grid line and considers up to sixteen nearby endpoints on that line, preventing the shift from being replaced by a zero-offset straight motion. Requested lateral offsets are 1, 2 and 3 m; the actual offset can change to the closest representable grid line at an irregular heading. These endpoint-neighborhood details are implementation choices.

Rotational symmetry and the car model's forward/reverse symmetry reduce offline work. Straight primitives are exact analytic minimizers. All other retained primitives must have a successful native NLP flag and independently small shooting residual. Offline failures for discarded endpoint alternatives remain in the preparation history.

The heuristic lookup stores exact obstacle-free graph costs for every initial/final heading pair and every final XY grid location in a 40 m square, giving a `41x41x16x16` array. Dijkstra explores beyond that square until all required labels are finalized. It fails instead of silently accepting values truncated by its resource boundary. Online A* uses the lookup inside its range and Euclidean distance outside it, permits state reopening, and uses weight one. Thus the two heuristic regimes need not be assumed consistent.

## Online search and improvement

The problem is expressed in a rigid local frame at the actual start pose. This places the start exactly on the lattice without altering any geometry. Following Remark 1, an off-lattice goal is mapped to a nearby free lattice state and restored to the precise task pose by the improvement OCP. Candidates are the 3x3 position neighborhood around the rounded goal and headings within 45 degrees; the selection metric is squared XY error plus squared minimum-turning-radius-scaled heading error. The paper does not prescribe that metric. An off-lattice correction loses the paper's feasible-warm-start guarantee, as the paper explicitly acknowledges.

The vehicle is covered by **three circumscribed circles**, as in the paper's car experiments. Each rectangular obstacle is covered by circles circumscribing contiguous rectangular cells along its longer dimension; the circle count is the ceiling of the aspect ratio. This conservative adaptation provides a full cover of the supplied polygons. It can block a benchmark endpoint even when the true rectangle is collision-free; that is a method failure, not proof that the benchmark is infeasible. The requested multi-disc extension for STC/LIOM is not silently substituted here.

Primitive collisions use the same circles as the OCP, sampled at most every 0.05 m, with a disclosed 0.01 m clearance. Search vertex bounds cover the task and obstacle centres plus 10 m; the search limits are 180 s and 500,000 expansions. Intermediate primitive geometry is not clipped to the vertex sampling bounds.

After search, each primitive supplies one OCP phase. The modes remain fixed; all intermediate poses and nonnegative phase lengths are optimized, so redundant phases may shrink to zero. All five states are continuous across phases, including at gear changes. The cost and vehicle model remain identical to the primitive-generation problem. Native NLP failure is returned as failure; the unoptimized lattice route is not relabelled as an optimized result.

## Transcription and solver choices

The paper permits direct optimal-control methods and lists both Ipopt and WORHP; its reported experiments used CasADi/WORHP. This implementation uses AMPL/Ipopt with MA97. It therefore **does not claim to reproduce WORHP timings or convergence**. Each primitive uses sixty shooting intervals with constant `u`. Steering propagation is analytic; RK4 integrates position and heading against that steering polynomial. Simpson quadrature evaluates the objective. All these choices are shared by primitive generation and improvement.

The improvement keeps the primitive intervals when concatenating phases. Its node count is consequently `60*number_of_primitives+1`, rather than forcing every problem onto 200 nodes. This deliberate choice preserves the numerical warm-start equalities, which are central to this method. Collision constraints are imposed at shooting nodes; dense execution safety is measured separately. Ipopt tolerance and acceptable tolerance are 1e-9, with 2,000 iterations. Offline subproblems use an adaptive barrier and 120 CPU seconds; online improvement uses a monotone barrier and 240 CPU seconds. The separate process wall limits are 270 and 510 s, respectively. This solver setting was selected during development after the adaptive barrier reached its time limit on paths with collapsing phases. Each phase is bounded above by 200 m. These are implementation resource limits, not values claimed from the paper. Numerical-library threads are fixed to one.

## MATLAB interface and verification

`RunPlanner('LatticeOCP',caseId)` returns a path: uniformly spaced mileage within each gear run, exact cusp indices, XY, heading, steering and interval gears. The evaluator supplies timestamps through the common path timing rule. The planner's spatial `omega` and `u` remain in native diagnostics and are not mislabelled as time-domain controls.

`BuildLatticeGraph` performs the one-time native module build with a configured MATLAB C++ compiler. `BuildLatticeGraph('path/to/zig.exe')` supports a portable Zig 0.13 compiler on Windows. Binaries and caches remain outside the source tree. `BuildLatticePrimitives(outputFile)` and `BuildLatticeHeuristic(libraryFile,outputFile)` reproduce the offline data; this preparation is reported separately from online planner timing. `COMMONPARKING_LATTICE_DATA` can select an external offline-data folder.

`TestLatticeGraph` checks exact analytic shortest paths, an obstacle detour and heading transitions. `TestLatticePrimitives` independently integrates all 480 original five-state ODEs at sixteen times the shooting resolution, checks endpoint/grid connections, steering limits, nonzero side shifts and objective quadrature. It does not simply re-evaluate the transcription used by the NLP solver. `TestLatticeOCP` verifies native convergence, non-increasing matched cost, output endpoints and cusp spacing on known-feasible one-phase and forward/reverse two-phase problems.
