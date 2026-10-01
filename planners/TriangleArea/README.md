# TriangleArea — transcription of the printed 2015 model

Bai Li and Zhijiang Shao, *A unified motion planning method for parking an autonomous vehicle in the presence of irregularly placed obstacles*, Knowledge-Based Systems 86 (2015), 11–20, DOI [10.1016/j.knosys.2015.04.016](https://doi.org/10.1016/j.knosys.2015.04.016). The paper is distributed under CC BY. This is a new MATLAB/AMPL implementation of its equations, not the authors' original software or a reproduction of their reported experiments.

## Model distinction that matters for these results

The printed Eq. (1) explicitly uses the **front-axle midpoint** for `(x,y)` and states `xdot=v*cos(theta)`, `ydot=v*sin(theta)`, `thetadot=v*sin(phi)/wheelbase`. Eq. (5) places the rectangle relative to that front reference. Those exact equations are retained here. They are **not equivalent** to the standard rear-axle bicycle model used by CommonParking's evaluator. Converting front position to rear position does not remove that dynamics difference. Therefore, tracking failures or collisions in this table must not be interpreted as an isolated assessment of the triangle-area obstacle formulation.

The supplied later MATLAB example instead uses rear-reference `tan(phi)` dynamics and an Euler mesh. Substituting that model would conceal a difference from the cited paper, so it is not done silently. A corrected physical model would need an explicitly named adaptation and a new result table.

## Paper mechanisms

- Minimize the free final time, with five states and acceleration/steering-rate controls.
- Use the two-sided triangle-area test: every vehicle vertex is outside each obstacle, and every obstacle vertex is outside the vehicle. No circle approximation or OBCA replacement is introduced.
- Use fully implicit finite-element collocation with a cubic state polynomial and three control stages. The implementation selects right Radau points; the paper allows Gaussian or Radau collocation. `cp.Radau3` constructs the differentiation matrix from Lagrange polynomials.
- Retain free final steering. The benchmark requires an exact target pose, replacing the paper's terminal parking-box inequalities, as permitted by its Section 2.5 framework.

## Disclosed settings and adapters

The vehicle geometry and speed, acceleration, steering and steering-rate bounds are the common benchmark parameters. There are 66 finite elements, producing 199 unique trajectory samples; the paper's examples use 20 elements. This larger mesh follows the requested approximately 200-node benchmark configuration. Strict area inequalities are represented by a 0.01 m² excess. The final-time bound is 100 s.

The paper does not specify a reproducible initial-guess construction. This implementation uses raw Hybrid A* followed by rest-to-rest longitudinal timing, enlarges the initial duration by 1.5, and converts initial positions to the front reference. This is an initialization choice, not a stage attributed to the paper. Hybrid A* uses a 0.2 m / 0.2 rad grid, 0.7 m primitives, three steering samples, heuristic weight 3, reverse penalty 1 and gear-change penalty 5; its resource limits are 100,000 expansions and 180 s.

Ipopt uses the paper's `tol=1e-12` and `bound_push=1e-4`; acceptable tolerance is also 1e-12. MA97, 3,000 iterations, 120 CPU seconds and 270 wall-clock seconds are implementation settings. The process wrapper fixes numerical-library threads to one. It currently requires Windows MATLAB with .NET; a local AMPL/Ipopt installation is required and is not redistributed.

Exported XY values are translated to the rear axle by subtracting `wheelbase*[cos(theta),sin(theta)]`. The literal paper velocity, steering and controls are retained and are not claimed to satisfy the evaluator's bicycle equations. The public trajectory contains the unique collocation times, including both endpoints. Initial controls are extrapolated from the first quadratic control polynomial; later samples retain their stage values. The evaluator applies its published trajectory interpolation and tracking protocol.

## Success, limitations and verification

Success requires the native NLP to report `solved` with a successful numeric code and finite output. Native infeasibility, resource limits and numerical failures remain failures. Finite failed candidates are retained for optional plotting but are not scored.

The triangle-area test excludes vertex intrusion at the checked instants; crossing rectangles can still pass a vertex-only test. The paper's continuous-motion argument does not establish safety between a finite set of collocation nodes. This implementation preserves that mechanism and lets independent full-body evaluation reveal its limitations. `TestRadau` checks polynomial differentiation and quadrature and explicitly includes a crossing-rectangle example.

See [validation](../../results/TriangleArea/validation.json), [the twelve-case measurements](../../results/TriangleArea/metrics.csv), and the main README table. Native collocation defects and node collision counts are recorded separately from the evaluator's replay measurements.
