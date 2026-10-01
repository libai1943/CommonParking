# CommonParking

CommonParking is a MATLAB benchmark for terminal parking in 12 static, local scenes. The accepted scene geometry and start/goal poses are fixed.

## Development status

The public interface and evaluator are being rebuilt. This initial protocol document is not a claim that the implementation or result tables are complete. Tested planner releases will be added individually.

## Interface contract

`result = RunPlanner(plannerName, caseId)` will solve exactly one case; both arguments are mandatory and caseId is an integer from 1 to 12. Algorithm-specific settings belong inside the planner's own folder.

A result has exactly one representation: a geometric path or a timed trajectory. Paths preserve exact gear-change cusps and use uniform arc-length sampling within each gear run. Trajectories carry explicit timestamps and vehicle states/controls. Every result records the planner's own success flag and wall-clock computation time.

## Shared vehicle

Rear-axle reference point; metres, seconds and radians. Wheelbase 2.8 m, front overhang 0.96 m, rear overhang 0.929 m, width 1.942 m. Limits: speed magnitude 2.5 m/s, acceleration magnitude 1.0 m/s², front steering angle magnitude 0.7 rad, steering rate magnitude 0.5 rad/s.

## Evaluation protocol being implemented

An open, whole-horizon nonlinear optimal-control tracker follows the submitted reference, without obstacle constraints. Path submissions first receive time-optimal longitudinal timing between consecutive cusps, with zero speed at each cusp. The evaluator relaxes dynamic limits by 0.1%; vehicle geometry is unchanged.

Separate reported quantities are collision-frame percentage, terminal-pose attainment, executed duration, smoothness/control-effort components, and planner computation time. There is no aggregate score. Terminal tolerances are 0.01 m separately in x and y and 1 degree in heading, with heading compared modulo 2*pi. Collision sampling is at 1 ms, also including the exact final time.

The tracker is a numerical execution surrogate, not a physical closed-loop experiment. Input rejection, optimizer failure, interpolation, discretization, and metric definitions will be documented with the implementation.

## Reproduction policy

Each planner is implemented from its original paper, with documented adaptations where necessary. Planner folders use method names rather than survey reference numbers. Learning methods requiring training data and methods requiring unavailable expert rules are outside the current scope.

Priority: existing planners, Hybrid Curvature Steer with bidirectional RRT*, and tightly coupled lattice planning with optimal control. Up to 40 methods are planned; only completed and tested implementations will be listed as available.

Source models and MATLAB code will be public. Temporary solver jobs, private runtime binaries, caches and development archives are kept outside this repository.
