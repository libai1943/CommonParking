# Virtual protection frame trajectory optimization

Zhiming Zhang, Shan Lu, Lei Xie, Hongye Su, Dongliu Li, Qibing Wang and Weihua Xu, *A guaranteed collision-free trajectory planning method for autonomous parking*, IET Intelligent Transport Systems 15 (2021), 331–343, DOI [10.1049/itr2.12028](https://doi.org/10.1049/itr2.12028). [Publisher's open-access article](https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/itr2.12028).

`RunPlanner('VPF',caseId)` returns a trajectory. This is a handwritten implementation of the article's multiple-shooting model, segment constraints and frame-enlargement iteration. It requires the external AMPL/Ipopt runtime (MA97) and MATLAB Navigation Toolbox for Hybrid A* initialization. All online initialization, optimization and enlargement work is included in planner wall time. Numerical-library threads are fixed to one.

## Paper formulation

Equation (1) uses the rear-axle bicycle state `(x,y,theta,v)` and interval-constant inputs `(a,phi)`. Each of N equal time intervals is transcribed with one classical four-stage RK4 step, equations (7)–(8), and a multiple-shooting state equality. The objective is `N*h`, the final time. Both task poses are fixed modulo `2*pi`; speed is zero at both ends. The paper does not fix endpoint steering or acceleration, so this planner does not add those conditions.

The common benchmark dimensions and speed, acceleration, steering and steering-rate limits replace the article's example vehicle. The article's additional jerk bound of 0.6 m/s³ is retained. Bounds on jerk and steering rate use adjacent input differences divided by h; this is the finite-dimensional interpretation of equation (5), whose difference scheme is not specified. Because the native integration assumes piecewise-constant inputs, these difference bounds are **not a proof of continuous steering-rate or jerk feasibility**.

Definition 1 and equation (9) keep the frame length equal to vehicle length and multiply its width by `alpha >= 1`. Equation (16) checks twelve segments per interval: the four edges of each endpoint frame and the four connecting segments between corresponding corners. Shared frame edges are evaluated only once, leaving `8*N+4` distinct segments. Every such segment is tested against every polygon obstacle edge.

The Appendix A predicate is transcribed literally:

```
d1 = cross(A-C,D-C); d2 = cross(B-C,D-C);
d3 = cross(C-A,B-A); d4 = cross(D-A,B-A);
max(d1*d2,d3*d4) > 0.
```

The numerical model replaces strict positivity with a fixed `1e-8 m^4` lower bound. It retains the nonsmooth maximum; it does not substitute distance duals, discs or a smooth penalty. A native success requires Ipopt's solved flag and independent unscaled constraint residuals at most `1e-6` (in the respective equation units). Small residuals are not interpreted as exact mathematical separation.

Algorithm 1 starts at alpha=1, solves the NLP, computes the minimum enlarged frame satisfying equations (11)–(15) over all corners/intervals, and repeats if the required alpha exceeds the current one by more than `1e-3`. Signed curvature and signed radii are retained for both turning directions. `CoverDistance` algebraically rationalizes the radius difference and uses `2*sin(turn/4)^2` to avoid cancellation near straight travel. The straight limit is included explicitly. Bracketing and 60 bisection steps evaluate the printed cover inequality. The previous solution initializes the next enlarged-frame NLP.

As explicitly suggested in Section 3.3, Hybrid A* supplies the initial path. The benchmark's exact rest-to-rest longitudinal profile gives timestamps, then initializes the uniform shooting grid. The path uses the common vehicle's full curvature limit and 0.12 m search clearance. All search parameters are in `Config.m`; no other planner's successful output or stored solution is reused.

## Numerical choices and output

The benchmark run uses **40 intervals**, close to the article's three illustrated meshes of 35, 42 and 46. A development comparison on case 3 found that 200 intervals exhausted a 180 s NLP limit, whereas 40 intervals completed the two-step enlargement loop in about 29 s. This one uniform mesh is fixed for all 12 cases; no per-case mesh selection is performed. The final-time range is 0.1–80 s (the article's upper bound is 80 s; 0.1 s avoids a degenerate numerical interval). Each NLP uses Ipopt with exact AMPL derivatives, MA97, adaptive barrier updates, tolerance `1e-8`, at most 2,000 iterations and 180 CPU seconds. AMPL replaces the article's CasADi interface. The outer loop allows at most eight enlargements. A turn of magnitude pi or greater, or an alpha bracket beyond 4, is an explicit cover-calculation failure. None of these resource failures triggers an alternate planner.

The standard trajectory contains the N+1 state timestamps and `(x,y,theta,v)`, with interval controls `(a,phi)` appended by their final held value, and finite-difference `omega`. The common evaluator uses the documented linear reference interpolation and its independent physical execution model. Native interval-constant controls and their states remain in diagnostics for numerical inspection; they are not silently converted into a different optimized solution.

## Scope of the collision claim

This implementation reports native convergence and measured execution safety separately. It does **not** attach an unconditional collision-free certificate to the returned trajectory. The following details matter when applying the printed construction outside the article's examples:

- Appendix A's strict cross-product predicate rejects even disjoint collinear segments. This conservative degeneracy is retained, documented and unit-tested.
- The cover calculation uses net interval travel `(v*h + a*h^2/2)`. It does not constrain speed to keep one sign inside an interval. `TestVPF` constructs a straight interval with `v=-0.5 m/s`, `a=1 m/s²`, `h=1 s`: the two endpoint poses coincide, but the car first moves backward by 0.125 m. Every width-only endpoint frame misses this additional longitudinal excursion, although the printed algebraic cover test accepts alpha=1. This is a directly reproducible limitation of applying that test to intervals containing an interior reversal. The implementation retains the original interval formulation and records such reversals; it does not add undisclosed gear phases to repair the article.
- RK4 shooting endpoints differ slightly from exact constant-input integration, whereas the cover formula uses exact circular-arc geometry. The retained `1e-3` alpha stopping tolerance can also leave a positive cover distance at the frame actually used in the last NLP. These discrepancies are independently measured rather than assumed absent.
- Segment separation alone does not test polygon containment. The article assumes a collision-free initial configuration and a nearby initial path; this benchmark supplies both from its task and search. Release validation separately checks body footprints and interval replay without converting violations into successful safety certificates.

`TestVPF` checks the cross-product predicate, RK4 against adaptive exact-arc quadrature, signed-radius formulas in both directions, sampled frame coverage in monotone intervals and the interior-reversal counterexample. Release validation recomputes native constraints and objective, frame enlargement, exact interval integration, native footprints and dense between-node footprints. The unchanged common evaluator supplies the published collision percentage, terminal flag, execution time and smoothness components.

The released fixed-configuration run succeeds natively in cases 3, 4 and 7. All three have zero measured execution-collision frames; cases 3 and 7 meet the terminal tolerance, while case 4 does not. Each accepted native trajectory contains one interval with an interior speed reversal. Independent dense exact integration finds corner excursions outside the enlarged endpoint hull of approximately 0.245, 0.105 and 1.463 mm, respectively, without an actual obstacle intersection in these outputs. These observations are reported as limits of the frame-cover claim, not as additional collision events. The nine unsuccessful NLP runs are retained in the result table.
