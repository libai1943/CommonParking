# Primal separating-hyperplane optimal control

Jiayu Fan, Nikolce Murgovski and Jun Liang, *Efficient Optimization-Based Trajectory Planning for Unmanned Systems in Confined Environments*, IEEE Transactions on Intelligent Transportation Systems 25(11), 2024, pp. 18547–18560, DOI [10.1109/TITS.2024.3436045](https://doi.org/10.1109/TITS.2024.3436045). [Author's university record and manuscript](https://research.chalmers.se/publication/542487).

`RunPlanner('HyperplaneOCP',caseId)` returns a trajectory. The implementation realizes the article's **polytope/polytope separation formulation and single-car parking OCP**. It does not claim to reproduce its tractor-trailer, ellipsoid, lane-overtaking or feedback experiments. The 12 benchmark scenes contain polygon obstacles and no additional enclosing canvas, so only the applicable separating constraints are needed. AMPL/Ipopt (MA97) is required; there is no Hybrid A*, learned initialization or offline solution library.

## Equations and transcription

Section VII-A-5 explicitly reduces the article's tractor-trailer model to a single car with five states `(x,y,theta,v,phi)` and two inputs `(a,omega)`. This implementation uses that model with the common benchmark wheelbase, full rectangular body and motion limits. The two task poses are fixed modulo `2*pi`; speed and steering are zero at both endpoints, as in the paper's single-car experiment. Heading is not artificially restricted to [-pi,pi], which would exclude equivalent task poses.

Equations (25)–(27) introduce a variable final time and uniform normalized time intervals. One classical RK4 step integrates each interval, with constant acceleration and steering rate. Speed and steering therefore vary linearly inside the interval. The retained objective is

```
TF + integral(100*a^2 + 200*omega^2, t=0..TF).
```

These are the article's values `r=1`, `Q=diag(100,200)`. Since controls are constant within an interval, RK4 quadrature for this running cost reduces exactly to its interval length times the integrand. The comparatively large control weights can yield long parking durations; no time-optimality-only claim is made.

For each state node and convex obstacle, equation (12) adds a two-component normal `lambda` and a scalar offset `mu`. All four ego-body vertices satisfy `lambda'*ego >= mu`; all obstacle vertices satisfy `lambda'*obstacle <= mu`. Vehicle vertices are transformed by the full rotation/translation of equation (21). The normals and offsets are optimized jointly with vehicle states and controls. This is the primal hyperplane representation, without distance-dual multipliers or obstacle disc approximations.

Remark 1 replaces the strict nonzero-normal condition with a small positive norm bound. Here `lambda_x^2 + lambda_y^2 >= 0.01^2`; 0.01 is a disclosed numerical choice because the paper does not specify epsilon. No unit-norm constraint or positive clearance margin is added. The support inequalities admit tangency, which the common evaluator can count as collision. Independent checks report both raw residuals and support residuals divided by normal magnitude, to expose any numerical amplification from small normals. Node separation does not prove between-node separation.

## Initialization

The single-car experiment in Section VII-A-5 initializes positions and heading by linear interpolation and sets all other states and inputs to zero. This implementation follows that state/control initialization; heading uses the shortest equivalent angular branch. Initial final time is 50 s, the article's stated parking-time guess.

For the auxiliary hyperplanes, the implementation uses the article's proposed **geometry-based initialization in Section VI-B**, with `gamma=0.75` from Section VII-A-3: the normal points from the obstacle's area centroid toward the rear-axle sample, and the plane passes through `0.75*sample + 0.25*centroid`. The normal is initially normalized. If the two points coincide, the initial normal is `[1,0]` to avoid division by zero. This completes a degenerate case not specified in the article. These initial planes need not already separate a colliding straight initial path.

This combines the paper's single-car state guess with its proposed geometric plane guess. Its dedicated computational-size comparison instead initialized normals to positive constants and offsets to zero; that comparison-specific initialization is **not** the configuration used here. There is no path-search initializer, scene-specific intermediate waypoint, or successful-output fallback.

## Fixed numerical configuration and standard result

All cases use 200 shooting intervals, following the benchmark's requested common optimization scale; the illustrated tractor-trailer experiment uses 30. The physical body and limits come from `BenchmarkConfig`. Numerical safeguards are a final-time range of 0.1–600 s, Ipopt tolerance `1e-8`, 2,000 iterations, 180 CPU seconds and a separate wall limit of 390 s. The 600 s bound agrees with the common evaluator's input resource limit. AMPL supplies derivatives and replaces the article's CasADi interface. All these choices are fixed in `Config.m`.

Native success requires the solved flag and independently recomputed RK4, endpoint, input/state-bound and hyperplane residuals at most `1e-6` in the corresponding equation units. Native failures remain failures. There is one NLP attempt, no alternate initializer and no relaxation of collision constraints following a failed solve.

The N+1 output timestamps carry all five states. Interval acceleration and steering rate are appended by their final held value. The common evaluator constructs its standard reference interpolation and performs its own high-accuracy execution optimization. All initialization, NLP and standard-output work contributes to planner wall time; numerical-library threads are fixed to one.

`TestHyperplaneOCP` checks RK4 against fine integration, exact body area and the linear-state/geometric-plane initialization. Release checks additionally recompute all hyperplane constraints and objective, verify endpoints and standard fields, integrate every interval independently, and inspect full-body footprints at native nodes and a fine time grid. The published execution metrics use the unchanged common evaluator, rather than the planner's own geometric claims.
