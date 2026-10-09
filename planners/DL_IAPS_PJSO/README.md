# Dual-loop iterative anchoring and piecewise-jerk speed optimization

Jinyun Zhou, Runxin He, Yu Wang, Shu Jiang, Zhenguang Zhu, Jiangtao Hu, Jinghao Miao and Qi Luo, *Autonomous Driving Trajectory Optimization With Dual-Loop Iterative Anchoring Path Smoothing and Piecewise-Jerk Speed Optimization*, IEEE Robotics and Automation Letters 6(2), 439–446, 2021, DOI [10.1109/LRA.2020.3045925](https://doi.org/10.1109/LRA.2020.3045925).

`RunPlanner('DL_IAPS_PJSO',caseId)` returns a trajectory. This MATLAB implementation follows the article's equations (2)–(6) and Algorithm 1. The authors' [Apollo v6.0.0 source](https://github.com/ApolloAuto/apollo/tree/v6.0.0/modules/planning/open_space/trajectory_smoother) was consulted to resolve boundary directions and unspecified settings. The paper and that snapshot differ materially; the implementation does not claim to be an exact port of Apollo. No code from the Apollo repository is bundled.

## Path smoothing

An ordinary Hybrid A* search provides a collision-free initial path, without CG smoothing. Its exact cusps partition the path into constant-gear maneuvers. Each maneuver is sampled at uniform mileage no greater than 0.1 m, with at least twelve points. The search uses 80% of the vehicle's maximum curvature, leaving room for the discrete endpoint-direction constraints; optimization uses the full common limit. Development tests on exact maximum-curvature circles exposed this initialization sensitivity. This uniform search choice does not change the benchmark vehicle. Smoothing operates on rear-axle positions. Its objective is exactly the sum of squared second differences in equation (2a), with no added path-length or reference-tracking penalty.

The discrete curvature constraint retains **both terms of the quartic expression**:

```
g_k(P) = ||2 P_k - P_(k-1) - P_(k+1)||^2
       - kappa_max^2 ||P_k - P_(k-1)||^4 <= 0.
```

The entire expression is differentiated analytically and linearized at each accepted iterate. The QP has nonnegative L1 slacks, a penalty loop, a successive-linearization loop, and a trust-region loop. Its acceptance ratio compares the true penalized decrease with the linearized model decrease; rejected steps reduce the trust radius, accepted steps enlarge it. Position bubbles become inscribed axis-aligned boxes. Both the objective and quartic constraints are divided by a fixed fourth power of the maneuver's mean reference chord length for numerical scaling; this does not freeze or remove the optimized distance term.

After inner convergence, the outer loop checks the **complete rectangular vehicle** against every polygon at every path node. It shrinks the reference-centred bubble of each colliding node and solves again. This node-based test is the paper's mechanism; it is not a proof of collision avoidance between nodes. Search failure, QP failure, iteration exhaustion, collapsed intervals or unresolved collision are explicit native failures. No other optimizer substitutes a successful trajectory.

## Speed optimization and output

Each maneuver independently starts and ends at rest. Decision variables are distance, speed and acceleration at equally spaced times. Acceleration is linear in time between nodes, hence jerk is constant. The exact cubic distance and quadratic speed equations (4a,b) are equality constraints. The QP objective is equation (6a): squared distance remaining, acceleration and jerk. Speed, acceleration and jerk have bounds; a constant lateral-acceleration speed cap uses the maximum reconstructed curvature on that maneuver. The horizon uses equation (6f), with the declared expansion ratio. The paper defines n states indexed 0 to n-1, so their final time here is `(n-1)*delta_t`.

The spatial profile uses one-sided/centred finite differences for heading and curvature. The speed polynomial is evaluated analytically, while x, y, unwrapped heading and steering are interpolated in path mileage. Output includes every speed knot and exact gear-change time, supplemented to at most 0.02 s spacing. Acceleration is signed by gear; steering rate is the derivative of the interpolated steering with respect to mileage times positive mileage speed. At a shared cusp the next maneuver supplies the single right-sided sample. No artificial steering dwell is added. Independent spatial maneuvers may have steering jumps there.

The quartic constraint is a discrete approximation derived under densely and uniformly spaced points. It is not an exact continuous bicycle model. Reconstructed curvature can differ from its proxy, and this paper does not directly constrain steering rate. Those limitations are measured by the common execution evaluator; they are not silently repaired in this planner.

## Explicit interpretations and source differences

- The printed terminal heading formula (2d)/(3d) has a plus sign. Taken literally for forward motion, it puts the penultimate point beyond the goal and reverses the terminal tangent. The authors' `AdjustStartEndHeading` correctly uses the opposite direction into the goal, accounting for gear; that sign interpretation is used here. The first and last chord lengths remain free, as allowed by equations (2c,d): an affine normal equality and a nonnegative tangent projection express each prescribed direction exactly. The snapshot's stronger fixed-neighbour positions are not imposed. All free positions use the same finite bubble/trust bounds for numerical localization.
- The article describes a terminal stop but does not spell out final `(s,v,a)` in equation (6). The authors' `SmoothSpeed` fixes them to `(length,0,0)`; this implementation retains those constraints and the initial `(0,0,0)`.
- Apollo v6.0.0 adds position/reference penalties, uses a different large time-horizon heuristic, and its default configuration disables the curvature constraint. None of those changes replace the paper's objective, quartic constraint, trust-region algorithm or equation (6f) here. Its collision code can skip initially colliding nodes; this implementation checks every node.
- MATLAB `quadprog` replaces OSQP. Both are convex QP solvers; positive native flags and independently recomputed residuals are required. The solver backend and runtime are not presented as the authors' runtime.
- Gear-run partitioning before path smoothing is the adapter used for a multi-cusp task; the paper explicitly separates gear runs for speed planning. Global task endpoints and all cusp poses are retained.

## Numerical configuration

All vehicle dimensions and speed/acceleration/steering limits come from the common benchmark. The method-specific jerk and lateral-acceleration limits are 1 m/s³ and 1 m/s². Speed QP weights are distance 10, acceleration 1 and jerk 1, taken from the authors' configuration where the paper leaves weights unspecified. Its time step is 0.2 s and horizon multiplier is 1.5. The 0.1 m spatial spacing is from the paper's illustrated setup. Bubble radius begins at 2 m, shrinking by 0.9 after collision; 100 collision iterations, 1,500 points per maneuver and 180 s for all smoothing/speed work bound resources. Trust radius starts at 1 m, is capped at 2 m, grows by 2 or shrinks by 0.5, with acceptance threshold 0.1. The normalized merit penalty starts at 10 and grows tenfold for at most seven penalty levels, with at most 1,000 linearizations per level. Step/cost tolerances are 1e-6; normalized quartic feasibility tolerance is 1e-5. QP objectives are divided by the current penalty (when greater than one), an exact positive scaling for conditioning. All settings are in `Config.m` and uniform across the twelve cases. Search, path smoothing and speed optimization all count toward the reported planning time; numerical-library threads are fixed to one during the call.

`TestDLIAPS` checks the full quartic gradient by finite differences, rotated straight-path boundary preservation, and forward/reverse constant-jerk speed dynamics and terminal rest by independent quadrature. Release validation additionally recomputes every accepted path constraint, footprint test, QP dynamics and bound, cusp, exported state and task endpoint. No success on the full benchmark is implied by these unit tests.
