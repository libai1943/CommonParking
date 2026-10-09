# Evaluation protocol

`evaluation = EvaluateResult(result, caseData)` measures one submission. All public numerical choices are in `BenchmarkConfig.m`. Evaluation does not change scenes or planner results. There is no overall ranking or aggregate score.

## Provenance and deliberate changes

The starting point is the organizer's TPCAP final evaluation source and Li et al., *Online Competition of Trajectory Planning for Automated Parking: Benchmarks, Achievements, Learned Lessons, and Future Perspectives*, IEEE Transactions on Intelligent Vehicles 8(1), 2023, DOI [10.1109/TIV.2022.3228963](https://doi.org/10.1109/TIV.2022.3228963).

The original code used a fixed submitted timeline, backward Euler dynamics, fixed start and terminal poses, waypoint tracking, an optional tracking tube and a position-only fallback. Its collision term counted obstacle/frame pairs. CommonParking deliberately changes those choices: free time scaling, Hermite-Simpson transcription, a free terminal pose, no tracking tube, no hidden fallback objective, and a percentage of colliding **frames**. These are benchmark protocol choices, not claims about the old competition implementation.

## Screening

Only `path` and `trajectory` representations are accepted. The result must identify its case and schema, contain a boolean planner-success flag, and a finite nonnegative planner computation time. A planner-reported failure terminates evaluation immediately. Numeric arrays must be real, finite, correctly sized column vectors. Complex values are rejected, never silently converted with `real()`.

Paths must have increasing arc length from zero, segment gears of +1 or -1, and exact cusp indices including the two endpoints. Each gear run has uniform spacing, at most 0.05 m. Trajectories must have strictly increasing timestamps from zero, complete states and controls, and zero initial/final speed. Nonuniform time sampling is allowed. Negative coordinates, angles, accelerations and reverse speeds are valid.

An endpoint mismatch greater than 1 m in either coordinate or 30 degrees modulo 2*pi is rejected before optimization. This is a gross-error guard, distinct from the much tighter final attainment tolerance. Resource guards are 2,000,000 submitted samples, 600 s reference duration, 1,000 m path length and a 1,000 m radius around the start. These guards are not a score. Optional exact circular-arc geometry must agree with all exported poses, steering, mileage and gear intervals.

Failure codes distinguish `planner_failed`, `invalid_result`, `gross_task_mismatch`, `reference_resource_limit`, `tracking_failed`, `tracking_discretization_failed` and `evaluation_exception`. Failed evaluations have unavailable metrics (`NaN`), not invented numerical penalties.

## Giving a geometric path a timeline

Every uninterrupted forward/reverse run is timed independently. If its length is L, speed bound V, acceleration A and braking magnitude B, its peak speed is

`Vp = min(V, sqrt(2*L/(1/A + 1/B)))`.

Acceleration lasts `Vp/A`, braking lasts `Vp/B`, and any remaining distance is driven at Vp. This yields the exact triangular or trapezoidal minimum-time profile for the bounded longitudinal double integrator. Forward and reverse speed bounds and acceleration/braking magnitudes are represented separately. Speed is exactly zero at all cusps and both task endpoints. A steering discontinuity does **not** silently insert a stop or stationary steering dwell into the submitted path; the tracker must respond to it within its steering-rate limit.

For HA+CG, the retained circular-arc representation gives exact XY and heading at any arc length. External sampled paths without exact geometry use explicitly documented linear interpolation in arc length; submitted timed trajectories use linear interpolation in time, with heading unwrapped first. Thus interpolation is part of the public reference definition, not an undocumented smoothing step.

## Whole-horizon optimization

The tracker solves a deterministic kinematic bicycle optimal-control problem over the full maneuver. States are `(x,y,theta,v,phi)` and controls are `(a,omega)`, with `xdot=v*cos(theta)`, `ydot=v*sin(theta)`, `thetadot=v*tan(phi)/wheelbase`, `vdot=a`, and `phidot=omega`.

The true case start pose is fixed; initial and final speed and steering angle are zero. The final XY and heading are free. Obstacles are absent from this optimization. Geometry is unchanged and only motion limits receive a factor of 1.001.

There are initially 2,001 uniformly spaced phase nodes. With reference duration Tref, the optimized duration T lies in `[0.5*Tref, 3*Tref]`. The reference is sampled at `Tref*u` and the executed state at `T*u`. The objective is the trapezoidal phase average of `(x-xref)^2+(y-yref)^2+(theta-thetaref)^2`, plus `1e-4*(T/Tref-1)^2`, plus `1e-4/(N-1)` times adjacent squared changes in acceleration and steering rate. Positions use metres and angles radians. The time term resolves the ambiguity of a free-duration, phase-tracking problem; it is **not** a reported performance score. The time range and this regularizer can affect the results and are therefore versioned protocol parameters. This is global time scaling, not unrestricted per-waypoint time warping.

Hermite-Simpson dynamics and linearly interpolated controls are solved with AMPL/Ipopt (`tol=acceptable_tol=1e-8`, 2,000 iterations, 180 solver CPU seconds per mesh). The whole objective is multiplied by 10,000 and automatic NLP scaling is disabled; relative objective weights are unchanged. This avoids premature stopping caused by a tiny phase-averaged objective. Native success must be `solved` with a solve-result number in 0–99 and a successful process exit. A last iterate from a failed solve is not accepted.

[Ipopt targets a local NLP solution](https://coin-or.github.io/Ipopt/), so a successful tracking solve is not a proof of the globally closest possible execution. The fixed initialization, objective, duration range and numerical checks define the reproducible evaluation procedure. Improving those protocol choices requires a new protocol version and rerunning every compared result, rather than replacing selected rows.

## Independent execution and numerical resolution

After optimization, the controls are independently integrated from the true start using RK4, with a maximum step of 1 ms and a split at every control knot. The integrated poses must agree with collocation poses to within `1e-4` m/rad componentwise. Speed and steering extrema between knots are also checked analytically, since they are quadratic under linear controls. A bound excess over `5e-7` triggers refinement. The mesh is refined to 4,001, then 8,001 nodes; failure to meet these tolerances is reported as an evaluation failure. This check prevents merely interpolating a coarse infeasible trajectory and calling it a high-accuracy execution.

Collision frames are at `0, 0.001, 0.002, ...` seconds, plus the exact final time. The final interval can be shorter than 1 ms. For exactly 10 s there are 10,001 frames. A full rectangular body versus convex polygon separating-axis test includes edge intersections and containment. Boundary contact counts as collision. A frame overlapping several obstacles is counted only once. Zero measured collision frames is not a mathematical proof of continuous-time safety between frames.

The word “execution” means this ideal bicycle-model numerical surrogate. It does not mean a physical vehicle test, feedback robustness guarantee, tire dynamics, actuator uncertainty or localization noise.

## Independent metrics

| Field | Definition | Unit |
|---|---|---|
| `collision_percent` | 100 × colliding frames / all frames | % |
| `terminal_reached` | abs(dx), abs(dy) <= 0.01 m and abs(wrapped dtheta) <= 1 degree | boolean |
| `execution_time_s` | Optimized and independently replayed duration | s |
| `control_effort_integral` | Integral of `a^2+(v*omega)^2` | m²/s³, radians dimensionless |
| `steering_integral` | Integral of `phi^2` | rad²·s |
| `gear_changes` | Sign changes after excluding samples with abs(v)<0.01 m/s | count |
| `smoothness_cost` | 10 × control effort + 10 × steering integral + 5 × gear changes | conventional weighted index |

The last three weights are retained from TPCAP's `ComputeCostFunctionValue.m` and `InitializeParams.m`. The original `100*tf` term is excluded because time is reported separately. The smoothness index is not energy in joules. Its raw components are also published, so the conventional mixed-unit weighting is fully inspectable. No collision, terminal, runtime or other penalty is added to it.

`evaluation.success` means the submitted format and tracker passed the evaluation procedure. A successfully evaluated result can collide or miss the terminal tolerance. `result.status.success` remains the planner's own success report. Planner wall-clock time is recorded separately; evaluator runtime is not a metric.

## Runtime and diagnostics

The MATLAB code and optimization model are public. Install a legally obtained AMPL/Ipopt runtime separately and configure `COMMONPARKING_AMPL_DIR` or `setpref('CommonParking','AmplDirectory',path)`. Runtime executables and licenses are not redistributed. Temporary jobs live under `tempdir/CommonParking`, or under `COMMONPARKING_WORK` if configured.

`evaluation.debug` holds the reference, native solution, independent execution, numerical integration check, terminal error and collision mask. `PlotEvaluation` displays submitted and tracked paths and colors colliding footprints red. These diagnostics are separate from the metric vector.
