# SE(2)-aware quasi-time-optimal nonlinear MPC

Christoph Rösmann, Artemi Makarow and Torsten Bertram, *Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups*, ECC 2021, pp. 1583–1590, DOI [10.23919/ECC54610.2021.9654872](https://doi.org/10.23919/ECC54610.2021.9654872). [Author preprint](https://arxiv.org/abs/2006.03534).

`RunPlanner('SE2_NMPC',caseId)` implements the paper's local quasi-time-optimal OCP for one static parking task. SO(2) angle differences, uniform optimized time spacing, quasi-time cost and optimization-based obstacle distances are retained. The common five-state bicycle, rest boundaries and interval separation below are explicit benchmark adaptations. This does not reproduce the complete ROS stack, feedback cycles, dynamic-obstacle experiments or optional topology extension.

## Common vehicle and transcription

The rear-axle node states are `(x,y,theta,v,phi)`. The loaded case supplies body dimensions, wheelbase, speed, acceleration, steering and steering-rate limits. Both endpoint poses are fixed modulo `2*pi`; both endpoint speeds and front steering angles are zero. No state is overwritten after solving.

There are 200 intervals and 201 state nodes. A uniform step `h >= 0.001 s` is optimized. Pose equations use the trapezoidal/Crank–Nicolson transcription of the common rear bicycle:

```
dx/h = (v(k)*cos(theta(k)) + v(k+1)*cos(theta(k+1)))/2
dy/h = (v(k)*sin(theta(k)) + v(k+1)*sin(theta(k+1)))/2
wrap(dtheta)/h = (v(k)*tan(phi(k)) + v(k+1)*tan(phi(k+1)))/(2*wheelbase)
a(k) = (v(k+1)-v(k))/h
omega(k) = (phi(k+1)-phi(k))/h
```

Signed speed and steering are linear between their nodes, so the last two equations give their exact interval derivatives. These derivatives obey the common limits. The objective is the trapezoidal running cost `h*sum(1 + 0.005*(v(k)^2+v(k+1)^2))`. The coefficient 0.01 on squared speed is a disclosed numerical choice. The article permits arbitrary state/input models; these equations specialize that formulation to the benchmark and keep optimization variables and exported quantities consistent.

Heading differences and task boundaries use `atan2(sin(difference),cos(difference))`. Adding any integer multiple of `2*pi` to an individual native heading leaves physical expressions unchanged. The output uses a continuous unwrapped lift. This exploits the periodic-function realization described in the article; headings are not compared by ordinary unwrapped subtraction at the task boundaries.

## Interval obstacle separation

The body is the complete benchmark rectangle and the physical minimum separation is **0.01 m**. Section III-E permits generic robot shapes and optimization-based obstacle distances. The convex-polygon distance dual is used, with nonnegative obstacle and body multipliers and `norm(A'*lambda) <= 1`.

Each interval/obstacle pair has one shared world-space normal `n=A'*lambda` and separate body support multipliers at its two endpoint poses. Thus both endpoint rectangles lie on the same separating side. At both endpoints the dual separation must be at least

```
0.01 + rho*wrap(theta(k+1)-theta(k))^2/8
rho = max distance from the rear axle to a body corner.
```

The second term bounds corner rotation between the endpoints; it is computed from the actual heading change, not a fixed extra safety buffer. For a body corner `r`, the difference between its rotated position at a linearly interpolated heading and the chord joining its two rotated endpoints has norm at most `norm(r)*dtheta^2/8`. Translation is already linear. Sharing the normal therefore gives at least 0.01 m separation for every intermediate **linearly interpolated submitted pose**, up to solver tolerance. This includes all corners and hence the full convex body. It does not certify a different physical trajectory produced by the independent execution evaluator.

This interval constraint is a disclosed benchmark collision adapter, not a claim that the original article printed this particular bound. It retains the distance-dual mechanism and objective while addressing the gap between node-only avoidance and the benchmark's reference interpolation. The dynamic mesh still has 200 intervals.

## Initialization and finite budget

Ordinary Hybrid A* supplies a cold-start path with the common vehicle limits and 0.01 m search clearance. The shared rest-to-rest longitudinal construction supplies its initial timestamps; it is sampled at 201 states. Exact geometric dual certificates at interval midposes initialize the shared normals. For each normal, nonnegative body supports are constructed at both endpoint orientations. All initialization is included in planning time. No prior solved case, CG result or other planner output is used as a fallback.

External AMPL/Ipopt with MA97 solves the NLP at `1e-8` tolerance, at most 2,000 iterations and 30 CPU seconds. A wall guard allows twice the CPU budget plus 30 s. One numerical-library thread is used. Native success requires a solved Ipopt status, independently recomputed dynamics/boundary/limit/dual residuals below `1e-6`, and agreement between the reported and recomputed objective. Finite-budget failure is reported as failure; it does not establish infeasibility.

## Output and verification

Output is a trajectory with native timestamps, five native node states, and interval acceleration/steering rate followed by a final zero input sample. `theta` is unwrapped for the standard continuous representation. Evaluation uses the unchanged common reference interpolation and physical execution optimizer.

`TestSE2NMPC` checks forward and reverse problems with known minimum time, endpoint rest, and arbitrary integer `2*pi` shifts. `TestSE2Envelope` checks the corner-rotation remainder over signed turns and multiple angle lifts. Release checks recompute native equations, interval certificates, geometric rectangle separation along the submitted reference at 1 ms, and standard output fields. Native solve status, execution collision frames and execution terminal attainment remain separate results.

The planner requires MATLAB Navigation/Optimization Toolboxes and the external AMPL/Ipopt runtime with MA97. Runtime scratch files remain outside the repository. Recorded wall times include setup, case loading, search, initialization, optimization and output conversion; one run per case does not reproduce or establish a controlled comparison with the paper's timing measurements.
