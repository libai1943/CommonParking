# Timed elastic bands in distinctive topologies

Christoph Rösmann, Frank Hoffmann and Torsten Bertram, *Integrated online trajectory planning and optimization in distinctive topologies*, Robotics and Autonomous Systems 88 (2017), pp. 142–153, DOI [10.1016/j.robot.2016.11.007](https://doi.org/10.1016/j.robot.2016.11.007).

`RunPlanner('TEB',caseId)` returns a trajectory. This MATLAB implementation includes both the sampling-based homology exploration and the timed-elastic-band least-squares optimizer. It is a finite, static whole-trajectory adaptation of the paper's repeatedly invoked local planner, not the authors' ROS controller and not a hard-constrained bicycle optimal-control method.

## Equation mapping

The optimization variables are interior poses `(x,y,theta)` and one positive time increment per edge. Both endpoint poses are fixed by elimination. Equations (5)–(16) are implemented in `Motion` and `Residual`:

* The time objective is **sum(dt²)**, not the square of total time.
* Nonholonomic residuals are the cross products of the sum of consecutive heading vectors with the position increment.
* The minimum-radius proxy is chord length divided by the absolute wrapped heading increment. Straight edges have zero radius penalty.
* Signed speed uses the paper's smooth direction approximation, `100*projection/(1+abs(100*projection))`; yaw rate is wrapped heading increment divided by dt.
* Interior linear/yaw accelerations use twice the difference of adjacent interval velocities divided by the sum of their durations. Boundary accelerations use the prescribed zero initial/final velocities. The boundary denominator is the adjacent dt, consistent with the authors' [acceleration-edge implementation](https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/include/teb_local_planner/g2o_types/edge_acceleration.h).
* Every pose is checked against every polygon using the complete oriented vehicle rectangle. Exterior distances are exact Euclidean polygon distances. Overlap uses negative separating-axis penetration as a continuous signed extension; no circle approximation replaces the car. The configured minimum gap is 0.1 m.

All inequalities enter as squared hinge penalties. The nonholonomic weight is 1000, and the other weights are 1, following the paper's suggested starting values. **No penalty is converted into a hard constraint, and no penalty weight is increased until a scene passes.** Small violations, full-body overlap and deviations from exact bicycle dynamics can remain.

The shared speed, acceleration and minimum turning radius are used. The paper also bounds yaw rate and yaw acceleration, whereas the benchmark specifies front steering rate. The disclosed conversion is

```
yawRateMax = vmax*kappaMax
yawAccelerationMax = amax*kappaMax + vmax*sec(phiMax)^2*wmax/wheelbase.
```

These are necessary envelopes of the common bicycle model, not a replacement for the coupled steering-rate constraint. TEB does not impose that coupled constraint; the common execution evaluator does. `omega` in the exported benchmark trajectory always means front steering rate, never yaw rate.

## Topology exploration and retention

`Explore` implements Section 3.2.2 and Algorithm 1. Each planning cycle draws 15 accepted uniform points in a rectangle centered on the start–goal segment, 6 m wide and 1.1 times its length. There are at most 1500 rejection trials. Directed graph edges must be free of intersection with polygon obstacles and make an angle less than 60 degrees to the start–goal direction. This interprets the experiment's angular threshold as a cosine in the printed dot-product condition. A clipped half-plane line test checks each edge exactly for convex polygons. This coarse graph tests a point; oriented full-body costs are evaluated by TEB.

Depth-first exploration checks a direct edge to the goal first. It keeps at most three distinct classes and has a 20,000-visit resource cap. Coarse paths are initialized at no more than 0.3 m spacing, with tangent headings except for the fixed task headings. There is no Reeds–Shepp substitution for the graph edges.

`Signature` implements equations (20)–(22). Markers are polygon vertex centroids, which lie inside these convex benchmark obstacles. The paper's `a*b*(z-BL)*(z-TR)` marker polynomial is retained; the single-obstacle special case uses the explicitly permitted analytic choice `f0=1`, because the printed `b=0` otherwise makes all signatures zero. Coordinates and all residues are scaled by common nonzero factors for conditioning. Equality tolerance is 1e-7. The minimum-absolute angular branch retains its **sign**, as required by a complex contour integral and confirmed in the [authors' H-signature implementation](https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/include/teb_local_planner/h_signature.h); taking the positive magnitude printed inside equation (22) would not be a homology invariant.

Algorithm 2 expressly allows an initial trajectory from a global planner. This implementation supplies an ordinary Hybrid A* path, including exact initial cusp poses, without CG. Its complete computation time is included. A failed initializer does not prevent subsequent graph exploration. At each later cycle signatures are recomputed and duplicate classes are removed. Forward-edge admissibility rejects detours, while at least the lowest-cost candidate is retained so that a required reverse maneuver is not automatically discarded. The output is the minimum of the **full penalized objective** among the final candidates, not the shortest or safest candidate chosen afterward.

## Sparse LM and the static adapter

The paper's car example uses four mesh-adjustment/optimizer calls per planning cycle and five Levenberg–Marquardt iterations per call. Those values are retained. Since the benchmark requests one complete open-loop result, ten cycles are performed with the same start and goal; the car is not advanced between cycles. This is the declared static adapter, not a simulated closed-loop run.

`Optimize` is a handwritten sparse, diagonally damped LM solver. Eleven finite-difference color groups cover the local graph dependencies, and the normal equations are solved by MATLAB sparse factorization. Trial steps use a gain ratio and are rejected if they increase the stated objective or make any dt at most 1e-5 s. The latter is an explicit numerical safeguard for positive durations; it does not introduce a different time penalty. There are at most twelve damping trials per iteration. The gradient and relative step tolerances are 1e-7 and 1e-9. No g2o binary or code is bundled.

Each resize call makes one sweep: split intervals above 0.33 s at the midpoint or merge intervals below 0.27 s, preserving total time and both endpoint poses. This follows Section 2.5 with target 0.3 s and hysteresis 0.03 s. Pose counts remain between 3 and 500. This simple paper-level sweep is not the later ROS source's time-redistribution heuristic. Resizing can remove an original search cusp; the TEB optimizer is free to alter its motion sequence.

There is a 180 s optimization/exploration budget in addition to the initializer's 180 s cap. Exceeding it is a native failure. All routines use one numerical-library thread, and the random stream is fixed to `201701+caseId` and restored on return. LM stages must make a finite accepted descent step or meet their stationarity/step stopping condition; nonfinite stages and stages that cannot do either are rejected. No failed stage is silently replaced with the search result.

**`status.success` here means that the fixed-budget soft optimization completed and a finite standard trajectory could be exported.** It does not assert NLP convergence or feasibility. `solver.convergence_certificate=false` states this explicitly. This distinction is necessary for a finite-iteration least-squares local planner; the independent execution evaluator measures its outcomes.

## Whole-trajectory output

All optimized pose/time knots are retained exactly. Interior linear and yaw velocities are the arithmetic averages of adjacent finite-difference interval velocities; start/end velocities are zero. This follows the layout of the authors' [`getFullTrajectory`](https://github.com/rst-tu-dortmund/teb_local_planner/blob/noetic-devel/src/optimal_planner.cpp), using the paper's smooth signed-speed approximation. Front steering is inferred as `atan(wheelbase*yawRate/speed)`; rest endpoints use the adjacent interval curvature. An interior zero-speed/nonzero-yaw node cannot represent a bicycle state and causes an explicit export failure. Acceleration and steering-rate fields use nonuniform-time finite differences. Values are not clipped to common limits, and no synthetic dwell or post-optimization speed law is inserted.

This export is a sampled reference, not an exact integration of its controls. The common evaluator tracks it using the common bicycle model and its published tolerances, independently checks collision frames, and reports terminal attainment and effort. That distinction is particularly relevant near reversals and where soft constraints leave residuals.

`TestTEB` independently checks distance against known rectangle geometry, rotated SAT collision signs, segment intersections, signed homology and resampling invariance, discovery of two obstacle-side classes, all sparse derivative entries against dense finite differences, a known straight-band objective and descent, and resize conservation. Release validation additionally recomputes the whole objective, candidate selection, endpoint poses, positive time increments, export fields and native optimization history.
