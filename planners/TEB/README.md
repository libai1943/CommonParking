# Timed elastic bands with common bicycle states

Christoph Rösmann, Frank Hoffmann and Torsten Bertram, *Integrated online trajectory planning and optimization in distinctive topologies*, Robotics and Autonomous Systems 88 (2017), 142–153, DOI [10.1016/j.robot.2016.11.007](https://doi.org/10.1016/j.robot.2016.11.007).

`RunPlanner('TEB',caseId)` returns a trajectory. The implementation retains the article's topology exploration, variable-time elastic band, quadratic soft penalties and sparse Levenberg–Marquardt (LM) solver. **The five-state bicycle band is an explicit benchmark adaptation**, not the printed pose-only car model or the authors' ROS controller. The common evaluator checks execution independently.

## Common-model band

Each knot contains `(x,y,theta,v,phi)` at the rear-axle midpoint. Each edge has a positive duration `h`. Both task poses and zero speed/steering at both ends are fixed by eliminating those variables. Headings use a continuous lift; the terminal representative is equivalent modulo `2*pi`.

For `p=(x,y,theta)` and `f=(v*cos(theta),v*sin(theta),v*tan(phi)/wheelbase)`, the pose residual is

```
p[k+1] - p[k] - h[k]/2 * (f[k] + f[k+1]).
```

Speed and front steering are linear on each edge. Their exact slopes are `a=(v[k+1]-v[k])/h[k]` and `omega=(phi[k+1]-phi[k])/h[k]`. Thus the band uses a trapezoidal transcription of the common five-state model, without substituting yaw acceleration for front steering rate. Nodal speed/steering and edge acceleration/steering-rate bounds come from `caseData.vehicle`. Linear interpolation of speed and steering has no hidden interior extrema. No state or control is clipped when exporting.

The objective preserves the paper's **sum of squared edge durations**, plus quadratic dynamics residuals and squared hinge penalties for inequalities. Fixed coefficients in this benchmark are `1e5 for dynamics, 1e3 for motion limits and 1e4 for obstacle separation`. They are numerical settings for the adapted residuals, not coefficients claimed to be prescribed by the source article. All twelve cases use the same coefficients; they do not increase during optimization.

Full oriented rectangular footprints are checked against every obstacle at every knot and at the midpoint of every pose edge. Exterior distances are Euclidean polygon distances; overlap is represented by negative separating-axis penetration. The physical separation target is **0.01 m**. Midpoint checking reduces sampling gaps but is not a continuous collision certificate.

**All dynamics and motion/obstacle inequalities remain soft.** Finite residuals, limit violations and executed collisions are possible. A native success means that a finite band survived the configured LM stages and could be exported; `solver.convergence_certificate=false`. It does not mean hard-constrained feasibility or global optimality. Saved validation reports include dynamics defects, physical-limit peaks and dense reference collisions separately from the evaluator's metrics.

## Initial bands and topology exploration

The ordinary Hybrid A* seed uses common dimensions/curvature, 0.01 m footprint clearance, 0.2 m position cells, 5 degree heading cells, 0.04 m collision sampling, three steering samples and a 180 s / 100,000-expansion limit. It supplies poses at up to 0.3 m mileage spacing, including cusps. Initial edge times are the larger of 0.3 s and the edge's circular-arc length divided by 1 m/s. Signed interval speeds and geometric curvatures initialize the new state fields; boundary and reversal-node speeds are zero. Initial front steering at both task endpoints is zero. This is a guess, not an asserted dynamically feasible solution.

Sections 3.2.2 and Algorithm 1 supply the sampling/homology backbone. Each cycle draws 15 accepted points in a rectangle aligned with the start–goal segment, 6 m wide and 1.1 times its length, with at most 1500 sampling trials. Directed point-graph edges must avoid polygons and make less than 60 degrees with the start–goal direction. DFS checks the direct goal edge first and has a 20,000-visit cap. At most three distinct classes are retained. Polygon vertex centroids are the markers; the complex contour signature retains signed angular branches. Coordinates and residues are scaled consistently. Signature tolerance is `1e-7`, with the allowed constant analytic function for a single obstacle.

Algorithm 2 permits a global-planner seed. Failure of Hybrid A* does not prevent subsequent graph exploration. Coarse graph routes are initialized with tangent headings and fixed task headings. Signatures are recomputed each cycle; duplicate classes are removed. Forward-edge pruning retains at least the cheapest representative, so a necessary reversing trajectory is not automatically discarded. Selection uses the complete penalized objective, not evaluation outcomes or a post-hoc safest-path filter.

## Sparse LM and temporal resizing

The static adapter performs ten planning cycles, each with four resize/optimization calls. Each call has **five LM iterations** and at most twelve damping trials per iteration. The starting damping is `1e-3`; finite-difference step, gradient tolerance and relative step tolerance are `1e-6`, `1e-7` and `1e-9`. Only finite cost-reducing trial steps with every edge duration greater than `1e-5 s` are accepted. A stage that neither accepts descent nor meets a stationarity/small-step condition is rejected. A failed optimization is not replaced with the initial search path.

The handwritten solver uses column-scaled damped normal equations and MATLAB sparse factorization. Every graph residual touches at most two adjacent five-state knots and one time edge, so eleven finite-difference colors suffice. Scaling changes the numerical linear solve, not the stated objective.

Resizing splits durations above 0.33 s or merges durations below 0.27 s, within 3–500 knots. All five states interpolate on a split. Merging preserves both endpoint states and total time; an interior node between oppositely signed neighbor speeds is retained to avoid erasing a represented reversal. The optimizer remains free to change the motion sequence. There is a 180 s exploration/optimization budget in addition to the initializer's cap. One numerical-library thread is used; the fixed random seed is `201701+caseId`, and the caller's random/thread settings are restored.

## Output and validation

The returned `t` is `[0;cumsum(h)]`; `(x,y,theta,v,phi)` are exactly the optimized knot states. `a` and `omega` contain the outgoing edge slopes, with zero in the unused final control slot. No ratio of averaged yaw rate and averaged signed speed is used at a cusp. No time dilation, artificial dwell or alternate speed law is inserted after optimization. Linear reference interpolation and the unchanged common execution evaluator determine the published execution outcomes.

`TestTEB` checks full-body distances, SAT signs, topology signatures and discovery, all colored derivatives against dense finite differences, exact forward/reverse straight bands, rotated coordinates, equivalent heading lifts, the `tan(phi)` turning equation, native steering export, LM descent and resize conservation. `ValidateTEBCommon` independently rebuilds every candidate's objective using a separate scalar polygon-distance implementation; checks endpoint states, candidate selection, LM histories and all exported fields; and checks reference footprints at 1 ms. These native checks do not replace execution measurement.
