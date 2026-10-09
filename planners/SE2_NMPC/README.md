# SE(2)-aware quasi-time-optimal nonlinear MPC

Christoph Rösmann, Artemi Makarow and Torsten Bertram, *Online Motion Planning based on Nonlinear Model Predictive Control with Non-Euclidean Rotation Groups*, ECC 2021, pp. 1583–1590, DOI [10.23919/ECC54610.2021.9654872](https://doi.org/10.23919/ECC54610.2021.9654872). [Author preprint](https://arxiv.org/abs/2006.03534).

`RunPlanner('SE2_NMPC',caseId)` returns a trajectory. This implements the paper's **local quasi-time-optimal OCP realization**, Sections III-B/D/E/F and IV-A, for a single static benchmark call. It does not claim to reproduce the complete ROS navigation stack, feedback loop, dynamic-obstacle experiment or optional multi-topology extension in Section III-G.

## Model, grid and objective

The article explicitly permits arbitrary state/input models. This realization uses the benchmark's rear-axle midpoint and its bicycle equations, with three pose states `(x,y,theta)` and two inputs `(v,phi)`:

```
xdot = v*cos(theta), ydot = v*sin(theta), thetadot = v*tan(phi)/wheelbase.
```

This is a declared rear-reference specialization of the generic formulation, rather than silently interpreting the center-of-mass coordinates and velocity in the article's equation (15) as rear-axle quantities. The common rectangle dimensions and motion limits are used throughout. Speed and front steering are controls; acceleration and steering rate are bounded through control differences as in equation (8). There is no additional velocity-state integrator.

The objective is the article's quasi-time-optimal running cost:

```
h*sum(1 + 0.01*v(k)^2).
```

Crank–Nicolson equations use the **same constant control on both ends of each interval**:

```
(x(k+1)-x(k))/h = 0.5*v(k)*(cos(theta(k))+cos(theta(k+1)))
(y(k+1)-y(k))/h = 0.5*v(k)*(sin(theta(k))+sin(theta(k+1)))
wrap(theta(k+1)-theta(k))/h = v(k)*tan(phi(k))/wheelbase.
```

The static benchmark uses 200 intervals, following the common benchmark resolution preference. The paper's example begins with 50 intervals and adjusts their number during feedback operation; that repeated feedback adaptation is not invoked in this one-shot OCP. A single global uniform time step h is optimized, which Remark 1 states is equivalent to equal local time variables. The paper's lower bound h>=0.001 s is retained and no upper bound is imposed.

All interior control differences divided by h obey the common acceleration and steering-rate bounds. Previous control is zero with elapsed time 0.1 s, as in the first paper control cycle. The final nonoptimized control is zero and the final control difference is constrained. The benchmark additionally requires **v(0)=0** so the first exported velocity is at rest. This extra boundary condition is disclosed; the first piecewise-constant-control interval is consequently stationary. The initial front steering can be nonzero within the previous-control rate bound. Final exported speed and steering are zero. There is no undocumented trajectory repair after optimization.

## Angular geometry

Every heading difference, including both task boundaries, uses `atan2(sin(difference),cos(difference))`. Geometry and dynamics otherwise depend on heading only through sine and cosine. Thus arbitrary multiples of 2*pi at individual pose variables do not change the physical functions or constraints. The final task heading is not forced to one Euclidean representative.

AMPL/Ipopt stores a real-valued lift of each angle. Evaluating the periodic functions at `theta+increment` is identical to evaluating them after the paper's SO(2) retraction `wrap(theta+increment)`. The same holds for their local derivatives away from the pi branch cut. This uses the simplification described in Remark 2, rather than applying ordinary unwrapped subtraction to headings. AMPL supplies exact derivatives of these expressions to Ipopt. Returned headings are unwrapped only for the standard output's continuous plotting/reference convention; no pose is changed.

## Exact rectangular obstacle constraints

Section III-E permits generic robot shapes and optimization-based obstacle-distance techniques. The article's capsule in its particular experiment is replaced by the benchmark's complete rectangular body. Its **0.2 m minimum separation** is retained. Some benchmark endpoints may not admit that extra gap; those calls are explicitly reported as endpoint-clearance failures without changing the scene or shrinking the margin.

At every pose, a convex polygon distance dual represents the exact Euclidean separation condition. Nonnegative obstacle/body multipliers satisfy normal matching, norm(A'*lambda)<=1, and the dual gap>=0.2 m. This is an equivalent algebraic representation of equations (11)–(12), using the technique expressly cited in Section III-E, not a different objective or a circular vehicle cover. All obstacle nodes, including both endpoints, are constrained. Between-node clearance is not certified by this discrete OCP and is assessed by the common execution evaluator.

## Initialization and numerical choices

The paper relies on initial/warm trajectories in a navigation setting without specifying one universal cold-start parking procedure. Ordinary Hybrid A* supplies this benchmark's cold-start path, and the shared exact one-dimensional rest-to-rest speed construction supplies its timestamps. The path is sampled at the chosen 201 grid states. Exact geometric distance-dual certificates initialize the extra multipliers. Search and all initialization work are included in planner time. CG and previous solved scene results are not used.

The sparse NLP is solved by external AMPL/Ipopt with MA97, adaptive barrier updates, 1e-8 tolerance, at most 2000 iterations and 180 CPU seconds. A wall-clock guard allows twice that CPU budget plus 30 s. Numerical-library threads are fixed to one. These are disclosed backend choices: the article permits interior-point or SQP solvers but does not prescribe these complete settings. Both a successful native Ipopt status and independently recomputed discrete feasibility at 1e-6 are required. Search failure, native solver failure and endpoint-margin failure remain failures. There is no fallback to another planner's result.

The exported time/pose knots are the native ones. Speed and front steering are the interval controls, with a final zero-control sample. Acceleration and steering rate are the declared finite control differences. Piecewise-constant controls with finite-difference rate bounds are the paper's transcription; they are not claimed to be a continuously differentiable physical execution. The common evaluator independently constructs and tracks a continuous feasible bicycle execution.

`TestSE2NMPC` solves forward and reverse straight-line problems whose discrete minimum time is known analytically, and checks arbitrary 2*pi shifts of individual poses. Release validation also recomputes every dynamic equation, wrapped endpoint constraint, input/rate bound, distance dual, objective and output field. The benchmark's execution collision percentage and terminal flag remain separate from these native checks.
