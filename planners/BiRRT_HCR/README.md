# Bidirectional RRT* with Hybrid Curvature Rate Steer

Holger Banzhaf, Nijanthan Berinpanathan, Dennis Nienhüser and J. Marius Zöllner, *From G2 to G3 Continuity: Continuous Curvature Rate Steering Functions for Sampling-Based Nonholonomic Motion Planning*, IEEE Intelligent Vehicles Symposium 2018, pp. 326–333. [DOI](https://doi.org/10.1109/IVS.2018.8500653).

`RunPlanner('BiRRT_HCR',caseId)` returns a **path**. This implements the paper's HCR00-Reeds–Shepp steering function inside bidirectional RRT*: endpoints and sampled tree nodes have zero curvature and zero curvature rate. Between gear changes both curvature and curvature rate are continuous; curvature may jump at a cusp, as expressly permitted by HCR. The motion is not claimed to have continuous curvature through cusps. This is the HCR00 variant used for the paper's parking experiment, not its separate CCR variants.

## Geometry and equation mapping

The independent cubic-spiral implementation lives in `native/hcr_math.hpp`. Arc length `s` always increases along traversal; gear `d` is +1 or −1. Each piece stores `[signed_length, kappa0, sigma0, rho]` and integrates

```
kappa(s) = kappa0 + sigma0*s + rho*s^2/2
sigma(s) = sigma0 + rho*s
theta(s) = theta0 + d*(kappa0*s + sigma0*s^2/2 + rho*s^3/6)
x'(s) = d*cos(theta(s)),   y'(s) = d*sin(theta(s)).
```

Equations (7)–(18) produce triangular or trapezoidal curvature-rate profiles, selected by `kappaMax <= sigmaMax^2/rhoMax`. Their integrated position defines the HCR circle, offset angle and minimum deflection, equations (17)–(22). Position uses eight-point Gauss–Legendre quadrature, subdivided to limit each piece's absolute polynomial phase bound to 0.5 rad. Heading, curvature and curvature rate use their exact polynomials. The ordinary and irregular turn lengths follow equations (23)–(27).

Elementary CCR turns use **both** constructions in Figure 8. The scalar unknown is `sigma0` for the three-spiral half or `rho0` for the two-spiral half; we solve the projected midpoint equation (28) with 65 bracketed bisections. Bounds on curvature, its rate and acceleration are checked. Equation (29)'s priority is retained: use an admissible type-II root first, then type I, then the ordinary/irregular full turn. The reverse half has reversed curvature profile and negated rate. `elementary_i` in the MEX gateway is an isolated validation entry point; planning always uses the paper's type-II-first rule.

The circle/tangent family construction comes from the authors' [Apache-2.0 steering_functions](https://github.com/hbanzhaf/steering_functions/tree/79b9217697b681d953768ea0d95a9307bd312a0e), commit `79b9217697b681d953768ea0d95a9307bd312a0e`. This public library implements **HC**, not HCR. Its HC00 family geometry is extended here with the new circle parameters, cubic spirals and elementary roots; merely renaming HC is insufficient. The licensed original family enumeration and shortest-family selection are retained, with all source notices and inherited BSD notices. Changes to the shared state, circle and turn files are identified in their source comments. The MATLAB tree implementation and MEX gateway are benchmark code, not the authors' ROS planner.

## Search, costs and collision handling

Two trees alternate uniform configuration samples with a 5% opposite-root bias. Nearest and neighboring nodes use HCR00 connector length. The near radius is `6*(log(n+1)/(n+1))^(1/3)`. Parent choice compares complete path costs; rewiring updates every descendant and never creates an ancestor cycle. A new node is connected to the opposite tree's near set. Reverse traversal negates gear, preserves physical curvature, and transforms the curvature-rate polynomial exactly. No Hybrid A* initialization or fallback is used.

The paper gives a **5 s initial-solution deadline**, then **3 s additional improvement**, retained here. An initial path completed after the deadline is rejected. In-flight edge operations finish before a budget check, so returned wall time can exceed the deadline slightly. Map construction, MEX loading, search and output conversion all count in the public planner time. The one-time compilation does not. We publish one seeded run per case, not the paper's 100-run statistical experiment. Wall-time stopping and machine load affect reproducibility.

Full physical rectangles are tested against convex obstacles using GJK distance. The paper's 0.1 m hard clearance is retained. Edges are checked every at most 0.1 m and at every cubic/arc/straight boundary, matching its sampled collision approach. No continuous swept-body guarantee is claimed; a separate dense rectangle check is included in release diagnostics. The common evaluator uses the actual body, independent of this planner's margin.

The paper names path length, cusps, curvature and inflated-grid footprint costs, but does not publish a complete weighted expression or grid kernel. Our explicit choice is

```
J = L + 2*number_of_cusps
    + integral(abs(kappa),ds)/(kappaMax*L)
    + integral(footprint_cost,ds)/L.
```

The curvature integral is exact for each quadratic piece, splitting at real zeros. Grid resolution is the published 0.075 m and inflation radius is 0.25 m. Each grid centre gets the chosen linear cost `max(0,1-distance_to_obstacle/0.25)`. Footprint cost is the maximum cost over grid cells intersecting the complete rectangle, including partial corner intersections by separating axes. Its path integral uses trapezoidal quadrature on the collision grid. There is no additional minimum-clearance cost. Complete-path normalization makes this objective nonadditive; we do not claim standard RRT* optimality guarantees for this finite implementation.

## Parameters and common-vehicle adapter

| Parameter | Value / source |
|---|---|
| Vehicle body and maximum curvature | Frozen common vehicle, not the article's different vehicle |
| Maximum curvature rate | `wmax/(wheelbase*vmax)` = 0.0714286 m⁻²; common steering-rate adapter |
| Maximum curvature acceleration | 0.3905 m⁻³, article Table III; the benchmark has no separate bound on this spatial derivative |
| Hard clearance | 0.1 m, article |
| Collision sampling | ≤0.1 m plus exact primitive boundaries |
| Grid / inflation | 0.075 / 0.25 m, article |
| First solution / improvement | 5 / 3 s, article |
| Goal bias / near-radius coefficient | 0.05 / 6, article |
| XY sampling domain | Task endpoints and obstacle vertex bounding box plus 3 m; chosen |
| Heading sampling | Uniform on [−π,π), zero curvature/rate at sampled nodes |
| Iteration guard / seed | 100000 / `67000+caseId`; chosen |
| Output | Uniform mileage separately within each gear run, ≤0.05 m, exact cusp locations |

Since `phi = atan(wheelbase*kappa)`, the chosen rate ensures `abs(phi_dot) <= wmax` at every curvature when `abs(v) <= vmax`. This statement applies **between cusps**. A path's cusp steering jump needs execution handling; we do not invent a finite-time instantaneous steering change. The evaluator creates the same rest-to-rest timing and tracking used for all other path planners. Its endpoint rest/steering constraints and all measured outcomes remain separate from native geometric success.

## Build and checks

Run `BuildHCRSteer` once with MATLAB's configured C++ compiler. Windows also supports `BuildHCRSteer('path/to/zig.exe')`. The build and cache live under `COMMONPARKING_HCR_DIR`, or the system temporary directory `CommonParking/hcr-steer/<arch>`, outside the source tree. No executable or compiler is distributed. MATLAB orchestrates planning, data export and evaluation; the cubic-spiral geometry is a MEX acceleration module.

`TestHCR` covers 320 random connections across both triangular/trapezoidal ramp regimes, 60 independent adaptive ODE integrations, reverse traversal, continuous within-gear curvature/rate, exact derivative extrema, both elementary root constructions, curvature integrals and grid footprint checks. Release checks additionally inspect the exact goal, all output fields, cusp locations, uniform mileage and dense full-body collisions for every successful case. Failures are published without substitution by another planner.
