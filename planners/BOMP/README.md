# BOMP

Shenglei Shi, Youlun Xiong, Jiankui Chen and Caihua Xiong, *A bilevel optimal motion planning (BOMP) model with application to autonomous parking*, International Journal of Intelligent Robotics and Applications 3 (2019), 370–382, DOI [10.1007/s41315-019-00109-z](https://doi.org/10.1007/s41315-019-00109-z). The paper is available under CC BY 4.0. This is a new MATLAB/AMPL transcription, not the authors' original C++ software or a reproduction of their original scenario results.

## Paper formulation and algorithm

The state is `(x,y,theta,phi)` at the rear axle and the inputs are `(v,omega)`. Eq. (1) gives the standard rear-reference bicycle equations. The objective is Eq. (7), `tf + integral(v^2 dt)`. **Velocity is an input: the paper does not impose an acceleration bound.** We retain that formulation and export acceleration by differentiating the velocity polynomial. The evaluator independently applies the common acceleration bound. We do not add an acceleration state silently.

Each obstacle has its own J₂ linear program from Appendix Eq. (11). With obstacle vertices `A`, vehicle vertices `B`, and `p=(r,s,zx,zy,zA,zB)`, its equalities are `A*r-B*s+[zx;zy]=0`, `sum(r)+zA=1`, `sum(s)+zB=1`; all components of `p` are nonnegative. The objective is `zx+zy+zA+zB`. This is a pseudodistance, not Euclidean metres.

The nonlinear problem implements Eq. (9) literally: primal feasibility `Q*p=b`, nonnegative `p` and `lambda`, `norm(c-lambda+Q'*nu)^2 <= epsilon`, `lambda'*p <= epsilon`, and `c'*p >= epsilon+delta`. The safety pseudodistance is the paper's `delta=0.05`. Disconnected obstacles are represented separately.

Algorithm 1 is followed with `epsilon=0.1,0.01,...,1e-8`. The initial guess is the paper's constant initial pose with zero steering, velocity, steering rate and all primal/dual variables. The initial duration is 20 s; the paper explicitly permits an arbitrary guess. Hybrid A* is not used. Each successful NLP initializes the next relaxation step. The stopping test is a relative objective change of at most 1e-5; the paper says “small enough” without specifying a numerical threshold. The maximum eight relaxations and the 100 s duration bound are disclosed resource choices.

## Actual pseudospectral transcription

The implementation uses **15 global Legendre–Gauss–Lobatto nodes**, retaining the node count in Algorithm 1. The paper omits the particular pseudospectral node family, so LGL is an explicit implementation choice. States and inputs are global degree-14 interpolation polynomials; the dense differentiation matrix enforces the model at every node and LGL quadrature approximates the running objective. This is not Euler, trapezoidal or piecewise finite-element integration.

`cp.LegendreLobatto` constructs the nodes, differentiation matrix, quadrature and barycentric weights. `TestPseudospectral` independently checks derivatives through degree 14, quadrature through degree 27 and global interpolation. Public trajectories evaluate the resulting global polynomials at intervals no larger than 0.02 s and include every original LGL node. Acceleration is evaluated from the differentiated velocity polynomial. Interpolation does not change the native optimization mesh or make its between-node constraints exact.

The mesh is deliberately disclosed because a small global mesh can miss collisions, overshoot state/control bounds, or have substantial dynamics defects between its nodes. The public evaluator can expose all of these. Increasing the mesh would be a separate parameter study, not a reason to relabel this 15-node result.

## CommonParking adapters and numerical settings

The vehicle dimensions and applicable speed, steering and steering-rate limits come from `BenchmarkConfig`. The prescribed target pose replaces the paper's parking-box terminal tolerances. Heading uses the equivalent target angle nearest the initial heading; the evaluator still compares all headings modulo 2*pi. Initial steering is zero and terminal steering remains free. Both endpoint speeds are zero.

Ipopt settings follow Table 1: tolerance 1e-12, acceptable tolerance 1e-16, desired/acceptable constraint tolerance 1e-12, complementarity tolerance 1e-4, dual-infeasibility tolerance 1, and **MA86**. The installed Ipopt version is 3.13.4 rather than the paper's 3.12.9. Adaptive barrier selection, 3,000 iterations, 120 CPU seconds and 270 wall seconds per relaxation are implementation settings. Numerical-library threads are fixed to one. The bounded-process wrapper currently requires Windows MATLAB with .NET; AMPL/Ipopt/MA86 must be supplied by the user outside the repository.

An NLP failure terminates the continuation and is reported as failure, even if a preceding relaxed problem converged or a visually plausible candidate remains. A successful result requires both the native `solved` status and the objective-change stopping criterion. Failed finite candidates are available for debugging but do not enter evaluation.

## Verification and measured outcomes

All twelve cases were run from the specified constant initial guess. Cases 5 and 9 reached the native convergence condition. Their evaluator calls succeeded, but both tracked executions had collisions and neither met all terminal tolerances. The remaining cases report native infeasibility, iteration limits or numerical failures. These measurements concern this disclosed transcription and mesh, not a claim that every implementation or tuning of BOMP must have the same outcome.

The independent validation reconstructs every successful case's J₂ matrix, checks primal feasibility, stationarity, complementarity and relaxed safety, and solves fresh linear programs to report the actual J₂ values. It also checks the pseudospectral equations and reports native-node collision counts, dense reference collision samples and control peaks. The evaluator's replay metrics remain separate. See [validation](../../results/BOMP/validation.json) and [the complete measurements](../../results/BOMP/metrics.csv).
