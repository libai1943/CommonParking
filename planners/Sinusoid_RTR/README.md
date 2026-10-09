# Sinusoidal steering with a disclosed RTR obstacle-planning adapter

Richard M. Murray and S. Shankar Sastry, “Steering Nonholonomic Systems Using Sinusoids,” *29th IEEE Conference on Decision and Control*, 1990, pp. 2097–2101. [DOI](https://doi.org/10.1109/CDC.1990.203994), [authors' original conference paper](https://people.eecs.berkeley.edu/~sastry/pubs/OldSastryALL/MurraySteering1990.pdf).

The reproduced component is the **nonlinear three-stage car steering construction in Section 3, Example 2**, with the 1:1 and 1:2 sinusoidal frequencies. It is not a linearized small-angle car, a learned controller, or the later chained-form construction from the 1993 journal extension.

**The 1990 article does not supply a complete automatic obstacle-search algorithm.** It discusses shaping the input amplitudes/phases around obstacles and shows illustrative parking curves. To make that steering method usable on all twelve irregular static tasks, this benchmark adds the explicitly separate RTR geometric search from Kiss and Tevesz (2017), followed by recursive connection. This combination is called `Sinusoid_RTR`; results must not be presented as a reproduction of the 1990 paper's obstacle-planning experiments. The original method and this adapter are identified separately below.

## Nonlinear steering construction

The rear-axle position is expressed in a local orthonormal chart and divided by the common wheelbase `L`. Let these dimensionless coordinates be `X,Y`, let `alpha=sin(theta_local)`, and let `phi` be front steering. The paper's changed inputs give

```
dX/dtau     = v1
dphi/dtau   = v2
dalpha/dtau = tan(phi)*v1
dY/dtau     = alpha/sqrt(1-alpha^2)*v1.
```

The original physical model uses front-wheel speed, whereas the benchmark reference is at the rear axle. Both describe the same geometric car path: the physical rear speed is `L*v1/cos(theta_local)`, and front-wheel speed is this divided by `cos(phi)`. The unit-length coordinates above retain the wheelbase factor consistently; without normalization the alpha equation would contain `1/L`. `tau` is a construction parameter, **not an exported execution timestamp**.

All connection endpoints have zero steering. The stages are:

1. Apply constant `v1` with `phi=0` until `X` equals its target. Heading is unchanged and the exact induced lateral drift is retained.
2. Set `v1=a*sin(tau)`, `phi=b*sin(tau)` (`v2=b*cos(tau)`) for one period. `X` and steering return to their target values. The amplitude `a` is computed from the exact nonlinear integral of `sin(tau)*tan(b*sin(tau))` so that alpha attains its target. Its lateral drift is retained.
3. Set `v1=a*sin(tau)`, `phi=b*sin(2*tau)` (`v2=2*b*cos(2*tau)`) for one period. Exact harmonic symmetry returns `X`, alpha and steering to their target values; the nonlinear lateral integral determines the magnitude of `a` by a scalar bracketed root. Both signs of `a` are considered. No small-angle truncation is substituted for `tan(phi)` or `alpha/sqrt(1-alpha^2)`.

The original paper explains frequency selection but does not prescribe a unique amplitude-selection algorithm for arbitrary pairs. The disclosed rule here uses stage-2 steering magnitudes `min(phiMax,sqrt(abs(deltaAlpha)))`, both signs and half that magnitude. Stage 3 uses `min(phiMax,abs(deltaY)^(1/3))` and half that magnitude, with the sign of the required lateral displacement. Its scalar root uses 50 bisection iterations. These choices respect the common steering bound and shrink for nearby endpoint configurations. Up to sixteen candidates result; the shortest collision-free candidate is chosen.

The local chart points halfway between the two endpoint headings on their shorter angular lift. It requires `abs(alpha)<=0.98` throughout. A connection outside that chart or beyond the 1,000 m local-length limit is rejected and the global geometric path is subdivided. This is a finite numerical implementation; no global completeness or optimality claim is inferred.

`Harmonic.m`/`H.m` evaluate the exact period symmetries, with 24-point Gauss quadrature on the first quarter-period. Position and mileage use 32-point quadrature per quarter-period; the scalar lateral root uses 48 points. Full periods and zero-speed half-period boundaries are represented exactly. The nonlinear construction is checked independently by direct integration of the physical bicycle model.

## Explicit obstacle adapter

`rtr.Global`, shared with [RTR_TTS](../RTR_TTS/README.md), implements rotation/translation trees with TCI-interior nearest points, both directions of translation, blocked-rotation extensions and connections at TCI intersections. Its 3 m padded sampling box, 0.02 m geometric clearance, 10,000-iteration/120 s limits and continuous full-body collision logic are retained. The seed here is `1990068 + case_id`.

`Subdivide.m` tries the sinusoidal candidate family between endpoints of a portion of the geometric path. If every candidate collides or leaves the chart, that portion is bisected in translation-plus-weighted-heading mileage. Limits are 1,200 calls, depth 24 and 240 s. This adapter does not call Hybrid A*, the TTS/eeS connector or the CC-Steer family connector, and it does not substitute another solution when unsuccessful.

Each curved candidate is checked with the complete rectangular body. Full-body support-plane distance lower bounds and a body-point displacement bound `(1 + bodyRadius*kappaMax)*ds` certify the intervals between checked poses. Initial checking spacing is 0.15 m with every half-period boundary included; unresolved intervals are recursively subdivided to 1e-5 m and then rejected. A 1e-8 m numerical clearance guard is used. The underlying distance query comes from the licensed shared CC_PRM geometry module, not from a point/circle vehicle approximation.

## Standardized path output

`Index.m` separates every half-period at which longitudinal speed changes sign. `Sample.m` inverts strictly monotone unsigned mileage using an initial interpolation, safeguarded Newton iterations and bisection. `Path.m` then samples at uniform mileage within each complete gear run, at most 0.05 m apart, retaining every exact cusp. It exports rear-axle `x,y,theta`, front steering `phi`, interval gear signs and cusp indices. Heading uses a continuous lift; task agreement is modulo `2*pi`.

The result is a **path**. Construction amplitudes and periods do not imply a time-optimal or dynamically feasible traversal. In particular, steering is continuous but its derivative with respect to mileage can grow near a zero-speed cusp. The common path-to-trajectory and execution evaluator imposes the benchmark's acceleration, speed and steering-rate limits independently. Native and executed outcomes are kept separate.

All original phase parameters, exact local coordinate transforms and the geometric adapter path remain in diagnostics. No trajectory states are snapped to a target or replaced by a different planner after evaluation.

## Use and validation

```matlab
SetupCommonParking;
BuildCCSteer;   % shared distance-query module, one-time build
result = RunPlanner('Sinusoid_RTR',3);
evaluation = EvaluateResult(result,LoadCase(3));
```

The planner uses base MATLAB, the shared RTR global-search helper and the shared CC_PRM distance module. No nonlinear optimizer or Navigation Toolbox is used by the planner. The native-module source, attribution, portable Zig build and external binary cache are documented in [CC_PRM](../CC_PRM/README.md). The common execution evaluator still needs its separate AMPL/Ipopt runtime.

`TestSinusoid` checks eighty random/nearby connection queries, nonlinear endpoint and join accuracy, mileage/gear/cusp output, independently integrated physical bicycle dynamics, twenty-seven near-chart-boundary stress cases, shrinking maneuvers and a thin intervening obstacle. Release validation additionally integrates every submitted phase, checks complete-body separation continuously and at 0.005 m mileage, and independently verifies the output fields. Planning time includes global search, every local attempt, quadrature, recursive approximation and mileage conversion; the one-time module build is excluded.
