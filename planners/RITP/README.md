# Rapid iterative trajectory planning through differential flatness

Zhouheng Li, Lei Xie, Cheng Hu and Hongye Su, *A rapid iterative trajectory planning method for automated parking through differential flatness*, Robotics and Autonomous Systems 182 (2024), 104816, DOI [10.1016/j.robot.2024.104816](https://doi.org/10.1016/j.robot.2024.104816). [Author manuscript](https://arxiv.org/abs/2508.17038), [author implementation](https://github.com/zhouhengli/RITP/tree/f044723475d7ee7d02ded596e763ed7c995ea842).

`RunPlanner('RITP',caseId)` returns a trajectory. MATLAB Optimization Toolbox and Navigation Toolbox are required. The planner has no AMPL or Parallel Computing Toolbox dependency; the common evaluator uses AMPL/Ipopt. Search, polynomial optimization and conversion all count toward planning time. Small gear phases run serially with one numerical-library thread, without creating a process pool.

This implementation retains the paper's weighted polynomial path QPs, collision-driven reference reweighting and quintic parameter-time QP. The common-vehicle adaptations below are explicit; this is not a literal transcription of every printed boundary formula or an author-certified reproduction.

## Polynomial geometry and collision iteration

Ordinary Hybrid A* supplies a path, split at exact gear changes. Search uses 80% of the common maximum curvature and 0.01 m clearance; output geometry is checked against the full common vehicle limits. The curvature reserve is an initializer setting, not a change to the benchmark vehicle. Reference spacing is at most 0.1 m. Chord lengths define the parameter `s` and total reference length `L`.

Each gear phase uses polynomials `x(s), y(s)` in normalized monomials `s/L`. The path objective combines reference-position errors and first/second derivative penalties with weights `[50,0.1,0.5]`. End positions and both components of the unit tangents are fixed:

```
r(0) = start_xy;                  r(L) = goal_xy;
r_s(0) = gear*[cos(theta0),sin(theta0)];
r_s(L) = gear*[cos(thetaf),sin(thetaf)];
```

These endpoint derivatives enforce the prescribed headings for either gear without overwriting the resulting heading. Exact tangents replace the printed x-only inner alignment constraint. Degree 9 is tried first; degree 10 is attempted if the first fit fails collision, regularity or physical checks. This bounded polynomial-family choice is motivated by the later author code, which permits degrees through ten. It is uniform across all cases. The benchmark does not substitute another path planner after a failed fit.

At `max(15,round(20*L))` samples, full rectangular footprints are checked by separating axes with 0.01 m clearance. The checker includes crossing rectangles without contained vertices. On collision, the nearest reference point and an iteration-sized neighbourhood receive a weight multiplier of 1.2. At most 30 QPs are allowed per degree. The weighted fitting and collision iteration remain the core method; the degree range, iteration cap, complete polygon checker and exact endpoint tangents are disclosed benchmark adapters.

The flatness map uses `atan2(gear*y_s,gear*x_s)` and a continuous heading lift. Singular tangents fail. The native collision check is sampled; it is not a continuous collision certificate.

## Physical derivative mapping and time law

The velocity QP uses a quintic parameter law with six exact endpoint constraints. They uniquely determine

```
s(t) = L * (10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5).
```

The QP is assembled and its flag/residuals are checked. Its weights are `[1,0.5,0.3]`, with at least 15 time samples and a time/path sample ratio of 0.7. Initial duration is twice the rest-to-rest minimum time for parameter length `L` under the common speed and acceleration limits. This duration is a parameter, not a time-optimal solution.

The geometric parameter is not assumed to be true arc length. With `g=norm(r_s)` and `g_s=dot(r_s,r_ss)/g`, physical output obeys the rear-axle bicycle chain rule:

```
v     = gear * g * sdot;
a     = gear * (g*sddot + g_s*sdot^2);
kappa = gear * cross(r_s,r_ss) / g^3;
phi   = atan(wheelbase*kappa);
omega = phi_s * sdot.
```

A dense check of at least 2,001 time points rejects a phase with steering above the common 0.7 rad bound. A uniform duration increase, when needed, enforces the common speed, acceleration and steering-rate limits: velocity and steering rate scale inversely with duration, acceleration inversely with its square. The maximum required factor receives a 0.1% numerical reserve. No steering or state is clipped. Independent 1 ms native checks are included in release validation; neither finite check is claimed to prove all continuous extrema analytically.

At a cusp, position and heading agree exactly and both phases stop. If steering differs, the vehicle remains stationary while steering changes at at most 0.5 rad/s. Initial and final steering are zero by the same physical dwell construction. The added time is part of the submitted trajectory and execution-time measurement. Both sides of each dwell are retained, headings are lifted continuously, and nearly coincident sample times are merged before global phase offsets are applied. Moving output spacing is at most 0.02 s; a steering-only dwell is represented by its two endpoints and linear steering.

## Numerical settings and validation

All geometry and physical limits come from the loaded benchmark case. QP optimality/constraint tolerances are 1e-10, native residual tolerance is 1e-7, and the minimum tangent norm is 1e-7. Hybrid A* has a 180 s budget. `Config.m` contains the complete uniform settings. The source notice for consulted author material is in `3rd-party-licenses.txt`.

`TestRITP` checks QP objective equivalence, exact forward/reverse tangents, quintic coefficients, steering derivatives, physical velocity/acceleration chain rules and nonduplicated offset time grids. [ValidateRITPCommon](../../tests/ValidateRITPCommon.m) independently evaluates the polynomials with `polyval/polyder`, recompute the QP objective and projected gradient, check every phase mapping and dwell duration, and sample the native curve at 1 ms. The unchanged common evaluator separately optimizes execution and measures complete rectangular collisions and terminal attainment.

Ten of the twelve recorded calls pass native checks and both executed collision/terminal checks. Case 10 exhausts the polynomial collision iteration; case 12 exceeds the steering limit. These finite-budget failures do not establish scene infeasibility. See [all measurements](../../results/RITP/metrics.csv) and [independent validation](../../results/RITP/validation.json). Timing includes initialization and differs in scope from the article's optimization-only timing.
