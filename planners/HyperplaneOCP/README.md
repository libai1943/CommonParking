# Primal separating-hyperplane optimal control

Jiayu Fan, Nikolce Murgovski and Jun Liang, *Efficient Optimization-Based Trajectory Planning for Unmanned Systems in Confined Environments*, IEEE Transactions on Intelligent Transportation Systems 25(11), 2024, pp. 18547–18560, DOI [10.1109/TITS.2024.3436045](https://doi.org/10.1109/TITS.2024.3436045). [Author's university record and manuscript](https://research.chalmers.se/publication/542487).

`RunPlanner('HyperplaneOCP',caseId)` returns a trajectory. This is an independent implementation of the paper's primal polytope-separation OCP, adapted to the benchmark's single car and local polygon scenes. It requires AMPL/Ipopt (MA97). The initialization and interval constraints described below are explicit benchmark adaptations, not a claim of an exact reproduction of the paper's experimental setup.

## Model and objective

Section VII-A-5 reduces the paper's tractor-trailer model to five car states `(x,y,theta,v,phi)` and two inputs `(a,omega)`. The benchmark's rear-axle bicycle, complete rectangular body, wheelbase and motion limits are used. Both endpoint poses, speeds and steering angles are fixed; speed and steering are zero. Heading remains a continuous real variable. The endpoint representative is chosen from the search seed's continuous lift, so a target orientation equivalent modulo `2*pi` is accepted without introducing heading jumps.

There are 200 uniform shooting intervals and a variable positive duration. One classical RK4 step integrates each interval with constant acceleration and steering rate. Consequently speed and steering are linear within each interval. Equations (25)–(27) supply the time normalization and objective structure. The retained single-car weights are

```
TF + integral(100*a^2 + 200*omega^2, t=0..TF).
```

The running cost integrates exactly for held controls. These weights favor small control effort and can produce long maneuvers; this is not a minimum-time-only planner.

## Primal separation over an interval

Equation (12) separates the full vehicle polygon and each convex obstacle by an optimized plane: `n'*body >= mu`, `n'*obstacle <= mu`. There are no obstacle discs or distance-dual multipliers. Scaling a nonzero normal and its offset by the same positive factor leaves separation unchanged. The implementation fixes the redundant scale with `n'*n=1`; this is an equivalent geometric normalization of the paper's nonzero-normal condition, not an obstacle buffer.

One plane is shared by both ends of each interval. Let `rho` be the largest rear-axle-to-corner radius, `dtheta` the continuous heading change, and `h` the interval duration. Define endpoint maxima `V=max(abs(v_i),abs(v_{i+1}))`, `P=max(abs(phi_i),abs(phi_{i+1}))`, and `K=tan(P)/L`. These are bounds throughout the interval because speed and steering vary linearly. With held inputs `a,omega`, a bound on a physical corner's acceleration is

```
A = abs(a) + V^2*K
    + rho*(abs(a)*K + V*abs(omega)/(L*cos(P)^2) + (V*K)^2);
reserve = max(rho*dtheta^2/8, h^2*A/8) + 1e-5;
n'*endpoint_corner >= mu + reserve;
n'*obstacle_vertex <= mu.
```

The first reserve bounds the difference between a rotated corner and its endpoint chord when position and heading are linearly interpolated. The second also accounts for the rear axle's curved motion and angular acceleration under the common bicycle model. It follows by differentiating `rear_position + R(theta)*corner` twice and bounding the norm, then applying the interpolation remainder bound `h^2*max(norm(second_derivative))/8`. Both protect against geometry missed by independent endpoint checks. Nonnegative auxiliary variables bound the absolute speeds, angles and inputs; two smooth inequalities bound the reserve. No nonsmooth `max` is sent to Ipopt.

These are sufficient interval conditions added by the benchmark, and can be more conservative than the paper's node constraints. They are determined by the native motion, not a fixed centimetre-scale obstacle inflation. The only fixed strict support tolerance is `1e-5 m` (10 micrometres). The NLP adds no other physical buffer. The physical-motion argument assumes exact dynamics and exact separation; RK4 and the nonlinear solver introduce numerical errors, which are independently checked. Neither this discretization nor the independently tracked execution is advertised as a formal continuous-time safety certificate.

## Initialization and finite solver budget

The benchmark adapter initializes the OCP from ordinary Hybrid A* with 0.01 m search clearance. Search primitives are merged only when gear and curvature agree. At every curvature change the seed stops, changes steering under the common steering-rate limit, and traverses the next arc with an analytic bounded rest-to-rest longitudinal profile. The sampled seed has continuous heading and zero endpoint speed and steering. It is an initial guess, not a fallback output or a claim of exact RK4 feasibility after resampling.

For each interval, potential separating axes are computed from obstacle edges and both endpoint rectangles. The axis maximizing the support gap is normalized, and its offset is halfway between the corresponding supports. Zero-length edges are excluded, including those caused by standstill steering. These planes initialize the same jointly optimized primal variables. No case-specific waypoints, stored solutions or per-case parameter settings are used. This search-and-geometric initialization replaces the paper's illustrated straight, zero-control car guess; search time is included in planner wall time.

Every case uses the same 200 intervals, final-time range `[0.1,600] s`, Ipopt tolerance `1e-8`, 2,000 iterations and 60 CPU seconds per NLP. The process wall limit is 150 seconds. There is one NLP attempt. AMPL supplies derivatives in place of the article's CasADi interface. All settings live in `Config.m`, with physical values taken from `BenchmarkConfig`.

Native success requires the solver's solved flag and independently recomputed RK4, endpoint, bound, unit-normal and interval-support residuals no greater than `1e-6` in their corresponding equation units. A time or iteration limit remains failure even when an intermediate curve looks usable. Failure under these settings does not establish task infeasibility.

## Output and verification

The `N+1` timestamps contain native `(x,y,theta,v,phi)` states. Acceleration and steering rate contain the held interval inputs, with the final held value repeated at the endpoint. The unchanged evaluator interpolates the submitted reference and runs its own obstacle-free high-accuracy execution optimization and integration. Planning time includes setup, loading, search, seed construction, NLP and result conversion; evaluation time is excluded. One numerical-library thread is used. One recorded run per case is not a controlled runtime comparison with the original paper.

`TestHyperplaneOCP` verifies RK4 accuracy, full-body geometry, bounded seed controls and continuous headings. A rotating-rectangle counterexample verifies that two safe endpoint footprints can collide between them and that the rotation reserve rejects that interval. A separate adaptive-integration test checks the corner-acceleration bound. Release validation independently reconstructs all RK4 equations, objectives and primal support inequalities, checks the linear reference footprints at steps no larger than 1 ms, and independently integrates the held controls using adaptive integration. Native equation checks, reference separation, integrated replay and the common execution metrics are distinct reports.
