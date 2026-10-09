# LIOM refinement: short inner solves, sparse algebra and a fixed three-disc cover

The selected configuration solves 8 of the 12 unchanged tasks. All eight pass the unchanged common execution evaluation with **zero measured collision frames and terminal attainment**. Cases 3 and 6 fail the fixed-cover endpoint precheck; cases 2 and 10 exhaust the outer budget. All twelve outcomes, including those failures, replace the earlier LIOM table.

## Before and after

These are complete twelve-task suites, with one recorded run per task and configuration. `Both` is the intersection of terminal attainment and zero collision frames, not a composite score. Planning wall time includes failed calls but excludes evaluation.

| Configuration | Native success | Evaluated | Executed terminal | Zero collision | Both | Total plan (s) | Median plan (s) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Published baseline, 51 states, escalating covers | 9 | 9 | 4 | 9 | 4 | 3399.742 | 60.627 |
| Rejected trial, sparse algebra, short solves, 51 states, escalating covers | 11 | 11 | 5 | 9 | 3 | 373.248 | 17.149 |
| Selected, sparse algebra, short solves, 201 states, fixed 3 discs | 8 | 8 | 8 | 8 | 8 | 52.555 | 3.849 |

The selected result improves joint execution outcomes from 4 to 8, while native coverage falls from 9 to 8 under the stricter fixed-three-disc representation. It is not an improvement on every task or dimension. In particular, the three-disc cover cannot occupy the prescribed goal in case 3. The rejected 51-state trial increased native successes but produced poorer executed outcomes; it was not adopted. In that trial cases 6 and 8 had approximately 0.729% and 1.881% collision frames. Mesh density is therefore not hidden behind a claim about CPU limits alone.

**This is not a controlled CPU-limit-only speedup.** Sparse expression structure, inner termination handling, disc geometry, state mesh and shared setup overhead changed. Host load and one recorded trial add timing variability. The other 39 planners retain their existing records. No shared evaluator weights, tolerances, physics or obstacle geometry were changed.

## What caused the excessive cost

1. **Algebraically equivalent expressions produced very different solver sparsity.** The previous AMPL objective referenced aggregate defined variables for the entire nominal cost and infeasibility. In the tested AMPL/ASL path, this yielded a dense Hessian pattern. Expanding the same local terms directly in the objective preserves the mathematical problem and exposes the local structure. In the reproducible 51-state, three-disc fixture, the reported Hessian nonzeros fall from 149,535 to 3,000. Eight nontrivial assignments give a maximum objective difference of 1.07e-13. The penalty, dynamics and nominal weights are unchanged. See `TestLIOMFormulation` and the independent checks in the validation report.
2. **An inner resource limit is not necessarily failure of the LIOM outer method.** The new limit is 200 iterations / 2 CPU seconds per inner NLP, at most 10 outer iterations. A finite candidate returned with Ipopt limit code 400 or 401 may continue into corridor reconstruction only after independent hard-bound, endpoint and residual consistency checks. Large nonlinear residuals are allowed for continuation, never for success. Final success still requires the native solved flag and paper infeasibility below 1e-6; hitting a cap is not relabeled solved. The supplied author example also used a 2-second CPU cap and passed intermediate iterates forward.
3. **CPU caps alone do not bound all startup and serialization work.** A separate 10-second wall watchdog bounds each external inner solve. Ipopt checks its CPU budget during convergence checks, as documented in the [official options](https://coin-or.github.io/Ipopt/OPTIONS.html). The complete planner call also includes search, corridor construction and MATLAB work.
4. **Repeated path registration added unrelated MATLAB overhead.** `SetupCommonParking` now discovers folders on each call and adds only missing folders in one call. An isolated three-repeat measurement gave approximately 5.94–6.20 seconds for the legacy setup and 0.0081–0.0088 seconds for the replacement. This microstudy explains avoidable overhead; it is not a universal machine-independent cost. Other methods' saved times were not regenerated after this shared fix.

The original monotone-barrier settings and the 1e9 penalty are retained. Exploratory adaptive-barrier alternatives were not selected. Old cases 2, 6 and 10 accounted for approximately 88.9% of the baseline total planning time.

## Exactly three vehicle discs; 201 trajectory states

The vehicle is divided into three equal longitudinal rectangles. Each is circumscribed by a circle, with centres evenly spaced along the centreline. For the common 4.689 m by 1.942 m body, the rear-axle-relative centre offsets are -0.1475, 1.4155 and 2.9785 m; all radii are approximately 1.2464 m. The cover contains the complete rectangular body. A 0.01 m clearance is retained. There is **no disc escalation**.

The 201 states are temporal discretization points, not vehicle discs. The article's Table I uses 50 finite elements; this release uses 200 intervals to reduce transcription effects in execution. First-order explicit dynamics, the paper's fault-tolerant initializer, fixed penalty and iterative corridor reconstruction remain. This is a disclosed numerical-mesh and body-cover adaptation, not a claim of identical paper settings. The source mapping is in the [planner README](../planners/LIOM/README.md).

The fixed cover exceeds available endpoint clearance in two tasks: by approximately 6.92 cm at the goal of case 3, and 5.53 cm at the start of case 6. Those calls return `endpoint_cover_blocked` before search or NLP. This does not say the physical rectangular car or original parking task is infeasible.

## Native endpoints and executed endpoints are different measurements

LIOM's native X and Y terminal conditions are hard equalities. The paper's Eq. (19) represents final heading through a periodic sine/cosine penalty; the later supplied example uses a different hard-heading convention. This implementation retains the paper's periodic term and validates the complete penalty. None of the eight accepted outputs misses its native X/Y endpoint: their errors are exactly zero in the stored values. Maximum accepted wrapped heading error is about 7.55e-7 rad (0.0000433 degrees). We do not snap or repair a failed candidate's terminal pose.

The common evaluator fixes the initial pose, tracks the submitted trajectory without obstacle constraints, and stops at zero speed/steering. Its final pose is free, so its endpoint can differ from the planner's hard endpoint. The published `terminal_reached` flag tests that **executed** pose against the task: each coordinate within 0.01 m and heading within 1 degree modulo 2*pi. Maximum executed absolute coordinate error among the new eight successful cases is approximately 4.390 mm. The evaluator is a local ideal-model optimization surrogate, not a proof of globally closest tracking or real-vehicle behavior.

| Case | Planner status | Plan (s) | Native max XY error (mm) | Native wrapped heading error (deg) | Executed max XY error (mm) | Executed wrapped heading error (deg) | Executed collision (%) | Executed terminal |
|---:|---|---:|---:|---:|---:|---:|---:|:---:|
| 01 | solved | 3.1443 | 0.000000 | 0.000000164 | 4.390346 | 0.001223365 | 0 | yes |
| 02 | optimization_failed | 9.2176 | — | — | — | — | — | — |
| 03 | endpoint_cover_blocked | 0.0553 | — | — | — | — | — | — |
| 04 | solved | 2.2070 | 0.000000 | 0.000000075 | 3.697167 | 0.009525853 | 0 | yes |
| 05 | solved | 2.4297 | 0.000000 | 0.000000066 | 2.455782 | 0.004595355 | 0 | yes |
| 06 | endpoint_cover_blocked | 0.0162 | — | — | — | — | — | — |
| 07 | solved | 4.0850 | 0.000000 | 0.000043234 | 0.655069 | 0.009542434 | 0 | yes |
| 08 | solved | 3.6125 | 0.000000 | 0.000000093 | 0.804918 | 0.036559125 | 0 | yes |
| 09 | solved | 5.5034 | 0.000000 | 0.000000381 | 0.660600 | 0.000438070 | 0 | yes |
| 10 | optimization_failed | 12.5997 | — | — | — | — | — | — |
| 11 | solved | 4.6869 | 0.000000 | 0.000000214 | 2.911237 | 0.067538451 | 0 | yes |
| 12 | solved | 4.9977 | 0.000000 | 0.000000482 | 3.424692 | 0.051547059 | 0 | yes |

Unavailable execution metrics are not zero. Failed candidate endpoint diagnostics remain in JSON but are not interpreted as valid trajectories. Cases 2 and 10 reach the ten-outer-iteration budget without meeting the penalty threshold; their failure does not establish infeasibility.

## Reproduction and evidence

```matlab
result = RunPlanner('LIOM',1);
evaluation = EvaluateResult(result,LoadCase(1));
addpath('tests');
TestSetupCommonParking;
TestLIOM;
TestLIOMFormulation;  % requires configured AMPL/Ipopt runtime
```

[Current metrics](../results/LIOM/metrics.json), [independent validation](../results/LIOM/validation.json), and [complete before/after records and microstudies](../results/LIOM/refinement_study.json) are public. The baseline is commit `502421de44b46107603c1d8d77ee463e782e5884`. The archived coarse trial is evidence of a rejected configuration; only the fixed-three-disc 201-state outputs are the current published results. No benchmark case was edited.
