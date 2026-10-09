# Smooth canonical-curve planning

Florent Lamiraux and Jean-Paul Laumond, *Smooth Motion Planning for Car-Like Vehicles*, IEEE Transactions on Robotics and Automation 17(4), August 2001, DOI [10.1109/70.954762](https://doi.org/10.1109/70.954762). [Paper copy](https://www.cs.cmu.edu/~motionplanning/reading/Smooth_paths_for_carlikevehicle-1.pdf).

`RunPlanner('LamirauxSmooth',caseId)` implements the paper's **holonomic-path approximation** option in Section IV-A, with the canonical-curve local steering of Section III and random shortening of Section IV-C. The alternative PRM option of Section IV-B is not used. The output is a path with continuous steering, including at the joins between local curves; gear changes remain explicit cusps. The paper imposes a curvature bound, not a bound on curvature change per metre or on steering rate per second. Time parametrization and execution evaluation remain the common evaluator's responsibility.

## Paper construction

The state used by the local steering method is `(x,y,theta,kappa)`. A canonical curve `gamma(X,s)` follows the state's fixed curvature, so it is a circle or a straight line. We use rear-axle mileage `s` and `kappa=tan(phi)/wheelbase`. This is the geometric representation in Section II; the front-wheel velocity in its time-domain model does not change this path geometry.

For two states `A,B`, project the position of `B` onto `A`'s canonical curve and call its signed mileage `v2`. The local curve is

```
P(t) = (1-alpha(t))*gamma(A,v2*t)
     + alpha(t)*gamma(B,v2*(t-1)),       0 <= t <= 1.
```

Here `alpha(t)=10*t^3-15*t^4+6*t^5` is our choice satisfying the paper's endpoint conditions on alpha and its first two derivatives. Analytic first, second and third derivatives of P supply orientation, curvature and validation. Negative `v2` means reverse travel, so orientation and signed vehicle curvature are recovered with that gear sign. Regularity requires a nonzero derivative of P.

When this direct `Steer*` curve is not regular or violates the curvature bound, the paper's `Steer` construction selects a state on the goal's canonical curve, joins the initial state to it with `Steer*`, then follows that canonical curve back to the goal. The join shares position, orientation and curvature; opposite travel directions create a cusp.

The paper describes the reachable open set and existence of a suitable intermediate state, but does not give numerical constants or an executable rule for choosing that state. **This implementation searches a finite set of signed offsets on the goal's canonical curve.** It tries zero first, then increasing positive/negative offsets, and chooses the first continuously certified kinematically admissible candidate. Its collision test follows selection; a collision makes the outer approximation subdivide the failed interval. This finite search is a disclosed numerical interpretation, not a proof that the implemented connector inherits the paper's small-time controllability or completeness guarantees. Zero projection, singular curves, exhausted offset lists and exhausted certification depth are rejected explicitly.

Section IV-A first finds a collision-free geometric route in SE(2), lifts it with zero steering, chooses its midpoint and recursively connects the two halves. We use a bidirectional geometric RRT-Connect for the geometric route, with the same conservative continuous edge certificate as the geometric subdivision implementation. The article's potential-field/distributed-representation geometric front end is not reproduced; Section IV-A discusses alternative geometric planners. The lifted zero-curvature states and recursive smooth connection/curvature/collision checks are retained. Shortening draws two points uniformly by current path mileage, retains their actual curvatures, and accepts a shorter collision-free connection made by the same steering method. It does not replace this step with circular Reeds–Shepp or clothoid connections.

## Continuous checks and output sampling

`DerivativeBounds` encloses both planar components of P' and P'' over a parameter interval using interval products, the extrema of alpha derivatives, and Lipschitz bounds for sine and cosine. It tightens P' with its midpoint value and the bound on P''. These enclosures give a lower bound on speed, an upper bound on speed and an upper bound on absolute curvature. Floating-point padding is applied; this is conservative engineering arithmetic, not directed-rounding formal verification.

`Certify` subdivides intervals until regularity and the curvature bound are established. For collision checking, the midpoint full-rectangle separating gap must exceed the clearance plus a bound on all body-point movement, `speedUpper*(1+bodyRadius*kappaMax)*halfInterval`. Since each accepted interval also has a verified curvature bound, this bounds translation and rotation throughout it. An interval that cannot be certified within the depth limit fails. Endpoint-only or fixed-grid acceptance is not used.

Eight-point Gauss quadrature, adaptively refined on uniform parameter subintervals, provides a mileage table. Safeguarded Newton inversion of its local integrals gives samples uniformly spaced within each gear run, with every cusp exactly retained. The output spacing is at most 0.02 m. Analytic curve data remain in diagnostics; the standard submitted path contains `(s,x,y,theta,phi,gear,cusp_indices)`. It uses the evaluator's generic interpolation between these samples, not the optional circular-primitive representation. This sampling/interpolation distinction is part of the released interface.

## Numerical choices and limits

All these finite settings are implementation choices, not numerical values claimed from the 2001 article. Geometric planning: weighted pose step 1 m, goal bias 0.1, obstacle/task bounding box plus 3 m, clearance 0.12 m, 20,000 iterations or 40 s, certificate depth 18. Smooth subdivision: depth 20, 5,000 connections or 180 s. Curve checking: depth 18, 0.002 m clearance, minimum parameter speed 1e-8. For cusp selection, the scale is the maximum of 0.002 m, endpoint distance, square root of lateral displacement divided by maximum curvature, and heading difference divided by maximum curvature. Signed multipliers are 0.125, 0.25, 0.5, 1, 2, 4, 8, with absolute offset at most 12 m. Shortening: at most 10,000 trials, 1,000 consecutive failed trials, 60 s, minimum improvement 1e-5 m. The larger shortening budget was chosen after development runs revealed excessive maneuvers in parallel parking with 500 trials; it is applied uniformly to all cases. The seed is `101000+caseId`, with the caller's RNG restored. MATLAB's default numerical-library thread setting is retained.

All online stages count toward planner time. No guarantee of globally shortest paths, resolution completeness or a success probability is asserted for these finite choices. A bounded-search failure does not imply that a scene has no feasible solution.

`TestLamirauxSmooth` compares analytic derivatives with finite differences, interval enclosures with dense independent samples, endpoint and curvature joins, mileage inversion with MATLAB adaptive quadrature, and rejection of a thin obstacle between clear endpoints. Release verification additionally checks every final curve, joins, exact cusps, endpoint poses, monotone shortening and numerical mileage inversion on the exported result.
