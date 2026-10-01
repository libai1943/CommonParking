# Result format

Every public call is `result = RunPlanner(plannerName, caseId)`. Both arguments are required. One call solves one case. Algorithm settings are not public positional arguments; they belong in the selected planner's folder.

## Common fields

| Field | Meaning |
|---|---|
| `schema_version` | `CommonParking-result-1` |
| `planner`, `case_id` | Method name and integer case 1–12 |
| `kind` | Exactly `path` or `trajectory` |
| `reference_point` | `rear_axle_midpoint` |
| `units` | metres, radians, seconds |
| `status.success` | Planner's own boolean report |
| `status.code`, `status.message` | Machine-readable status and explanation |
| `computation_time_s` | Wall-clock time for the complete public planner call, including setup and case loading, excluding evaluation and plotting |
| `solver` | Native exit flags and method-specific stage status |
| `diagnostics` | Optional method-specific debugging data, not evaluation metrics |

All numeric solution arrays are finite real double column vectors. The inactive `path` or `trajectory` field is empty. The public call returns the result to the caller and does not silently save it, evaluate it or create figures.

## Path representation

`result.path` has N-element `s, x, y, theta, phi` arrays, an (N−1)-element `gear` array and `cusp_indices`. Arc length starts at zero and strictly increases. Gear is +1 or −1 for the interval after each sample. `cusp_indices` includes sample 1, every gear-change sample, and sample N. Each gear run is independently divided into equal distances no greater than 0.05 m; the last point of a run is always the exact cusp.

`phi` is the right-hand steering value at a sample, except the final sample uses its left-hand value. A steering discontinuity is allowed in a path. Heading is continuous as an orientation; comparison to task headings always wraps modulo 2*pi. No time or fictitious velocity is attached by the planner.

Optional `geometry.type='piecewise_circular'` stores `start` and an M×2 `primitives` array of signed length and curvature. This retains curvature changes inside equally spaced sample intervals. Geometry must agree with the exported samples and is validated before use. External planners may omit this extension.

## Trajectory representation

`result.trajectory` has N-element `t, x, y, theta, v, phi, a, omega` arrays. Time begins at zero and strictly increases; intervals need not be uniform. Speed is signed, steering is the front-wheel angle, acceleration is dv/dt, and omega is dphi/dt. The trajectory begins and ends at zero speed. Terminal time is `t(end)`. Controls at every timestamp, including the final one, must be provided. Piecewise constant controls can be represented using explicitly documented one-sided samples; the baseline evaluator uses linear interpolation between submitted timestamps.

## Evaluation result

`evaluation.success`, `code` and `message` describe whether screening and tracking succeeded. `evaluation.metrics` contains independent measured dimensions. Unavailable metrics are NaN. Collision and terminal-attainment outcomes do not overwrite the planner's own success flag. `evaluation.debug` is optional plotting/debugging information; it is not part of a scalar score.
