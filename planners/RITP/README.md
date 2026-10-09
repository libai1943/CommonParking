# Rapid iterative trajectory planning through differential flatness

Zhouheng Li, Lei Xie, Cheng Hu and Hongye Su, *A rapid iterative trajectory planning method for automated parking through differential flatness*, Robotics and Autonomous Systems 182 (2024), 104816, DOI [10.1016/j.robot.2024.104816](https://doi.org/10.1016/j.robot.2024.104816). [Author manuscript](https://arxiv.org/abs/2508.17038), [author implementation](https://github.com/zhouhengli/RITP/tree/f044723475d7ee7d02ded596e763ed7c995ea842).

`RunPlanner('RITP',caseId)` returns a trajectory. MATLAB Optimization Toolbox and Parallel Computing Toolbox are required. This is an implementation of the **printed 2024 algorithm**, with the numerical choices below. It uses no training data or offline path library. Hybrid A* runs on the requested scene, and its time is included. Constant-gear phases run independently with `parfor`, using at most four workers and one numerical-library thread per worker. If no pool exists, the planner creates a process pool sized to the phase count (capped at four) and closes it before returning. Pool startup and shutdown are included in wall time; an existing pool is reused and left open. The published fresh MATLAB runs therefore include substantial pool overhead and search, unlike the article's parallel optimization-only timings.

## Polynomial path and collision iteration

The searched path is split at exact gear changes. Each phase is sampled at at most 0.1 m intervals and chord distances define the reference parameter S, following equation (1). A pair of quintic polynomials represents x(s), y(s). Quintic is one of the degrees 3–5 studied in the article. Internally the polynomial uses s/L in [0,1], with derivative scaling by L, to improve conditioning without changing the objective or feasible polynomial family.

Equation (25) is solved with `quadprog`. The objective combines weighted squared errors at the reference points and first/second derivative penalties at a dense polynomial grid. End positions are fixed. The terminal smoothing constraints use m=2 and constrain **only the x component** of the two inner alignment points, as printed in equations (24)–(25), where b=[1,0]. No additional y alignment, exact tangent, steering-angle or curvature-continuity constraint is inserted. These finite-distance x constraints do not mathematically impose the exact endpoint heading, nor equality of curvature at a gear change.

Equation (6) and the flatness derivatives recover body heading and steering; `atan2` and the known gear resolve the quadrant and reverse-motion ambiguity of the displayed arctangent. Endpoint heading is the resulting polynomial tangent, never overwritten with the requested task heading. Undefined tangents are reported as a native failure. The polynomial parameter is not automatically the optimized curve's true arc length.

At `max(15,round(20*L))` samples, equation (2)'s **mutual vertex-in-rectangle test** is applied literally, including boundary contact. At the first sampled collision, the reference point nearest to that rear-axle sample is located, and nearby tracking weights are multiplied by beta=1.2. This is the value illustrated in Fig. 8; initial weights are one, as in Algorithm 1. The update window extends by the current iteration number on each side. The paper leaves its z value unspecified; this choice follows the later author code. At most ten QPs are attempted per phase, consistent with the article's experimental failure threshold. There is no higher-degree retry or alternate-path fallback.

Mutual vertex exclusion is weaker than polygon separation: it misses crossing rectangles when neither contains a vertex of the other. `TestRITP` preserves this counterexample. Native sampled acceptance is therefore not a continuous full-body safety certificate. The unchanged benchmark evaluator checks complete rectangle intersections at its execution frames.

## Velocity QP and output

Equations (26)–(35) use a quintic parameter law s(t), with zero speed and acceleration at both phase endpoints, s(0)=0, s(T)=L, and sampled bounds on absolute speed and acceleration. The six endpoint equalities uniquely determine

```
s(t) = L * (10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5).
```

The implementation still assembles and solves the stated QP, checks its solver flag and residuals, and tests against this independent closed form. Duration is a fixed hyperparameter, not an optimized variable. Here it is twice the exact rest-to-rest minimum time for distance L under the common speed/acceleration bounds. The factor two and velocity sample ratio 0.7 come from the author's public configuration. The number of time samples is `max(15,floor(0.7*M_s))`. Both endpoints are included exactly. There is no duration retry.

Standard output evaluates the geometric polynomial at s(t), supplies heading/steering from the flatness formulas, uses v=gear*s'(t) as explicitly stated in equation (35), a=gear*s''(t), and omega=phi_s*s'(t). **Because s is based on the reference length and the optimized polynomial need not have unit tangent magnitude, the reported speed can differ from the actual position derivative.** The code retains this limitation rather than silently changing the velocity formulation. No steering or steering-rate bounds appear in the printed QPs; violations are measured independently, not clipped away.

All gear-change timestamps and positions are retained. At a shared cusp timestamp the incoming phase's heading and steering are stored; the outgoing phase begins at its next positive time sample. Both one-sided values remain available in the native diagnostic phases. Finite tangent/steering jumps are reported in release checks. No dwell, cusp repair or extra smoothing is added. Initial/final speed and phase endpoint acceleration are zero to solver tolerance. The evaluator applies the same structural screening, reference interpolation and execution optimization as for every other trajectory planner.

## Numerical choices and difference from the later code

The paper omits numerical Q1–Q3 and R1–R3 values. This implementation uses the author's public configuration: path weights [50,0.1,0.5] and velocity weights [1,0.5,0.3], interpreted as scalar multiples of the identity. Minimum sample count 15 also follows that configuration. The Hybrid A* settings, 180 s initialization budget, polynomial regularity threshold `1e-7`, QP tolerances `1e-10` and native residual threshold `1e-7` are visible in the source. All are uniform across cases.

The author's repository explicitly notes changes from the article. Its pinned revision adds y alignment, allows polynomial degrees up to ten, uses beta=2, replaces equation (2) with Shapely polygon intersection, processes collision candidates differently, and remaps the velocity law through the smoothed sampled path's arc length. Those changes are **not** represented as the printed algorithm here. Only the otherwise unspecified settings listed above are borrowed. The source notice is retained in `3rd-party-licenses.txt`.

Native success means that every phase's path QP, sampled native collision termination and velocity QP succeeded. It does not imply compliance with the common evaluator's endpoint tolerance or kinematic limits. Independent release checks recompute objective/equality residuals, the time-law closed form, sampled polygon intersections, endpoint/cusp discrepancies and flatness fields. The public result table preserves all failures and execution outcomes separately.

The fixed release run returned all twelve native trajectories. All twelve execution optimizations succeeded with zero measured collision frames; cases 4, 6, 10, 11 and 12 attained the terminal tolerance. Independent dense native checks found zero body intersections, but the largest within-phase steering and steering-rate magnitudes were about 0.950 rad and 1.488 rad/s, above the common 0.7 rad and 0.5 rad/s limits. The largest discrepancy between the printed speed mapping and the actual position derivative was approximately 0.0498 m/s. These values concern this explicit paper-equation configuration, not the later author's modified implementation.
