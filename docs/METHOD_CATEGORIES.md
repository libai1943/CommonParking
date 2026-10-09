# Forty-method assessment by mechanism

This is a descriptive assessment of the **current CommonParking implementations and settings**, across the same 12 frozen microscopic terminal-parking tasks. There are 480 recorded calls: 353 planner successes, 352 successful evaluations, and 198 executions that both attain the terminal tolerance and have zero measured collision frames. The latter is a conjunction of two published checks, **not a weighted score**. It does not imply optimality or continuous-time collision certification.

Native success, evaluation success, collision rate, terminal attainment, execution duration and smoothness remain separate fields. Terminal tolerance is 1 cm in each coordinate and 1 degree modulo 2*pi; measured collisions use full-rectangle frames at 1 ms. The executor is a local, obstacle-free tracking NLP plus independent integration, not a physical vehicle or a proof of globally best tracking. See the [protocol](../evaluation/README.md).

## Explicit working taxonomy

The four groups below are mutually exclusive **bookkeeping choices**, not a claim that the algorithms have only one component. HA+CG, DubinsGrid and LatticeOCP are grouped by their search backbone; KinoDeform is grouped by its exploration trees. RTR_TTS and Sinusoid_RTR use obstacle-search trees but are placed with their constructive steering methods. Moving these hybrids to another group changes the pooled figures. The complete membership is listed below.

| Main mechanism | Methods | Calls | Planner success | Evaluated | Terminal attained | Zero collision frames | Both checks |
|---|---:|---:|---:|---:|---:|---:|---:|
| Analytic geometry and constructive steering | 5 | 60 | 58 | 57 | 47 | 46 | 41 |
| Sampling / roadmaps / exploration trees | 7 | 84 | 58 | 58 | 42 | 52 | 38 |
| Discrete search / value / reachability | 9 | 108 | 85 | 85 | 45 | 67 | 34 |
| Numerical optimization and optimization-dominant hybrids | 19 | 228 | 152 | 152 | 98 | 114 | 85 |

All outcome columns use the group's total call count as denominator; the last three count successfully evaluated executions only. These are counts from one recorded run per method and task. They are not estimated sampling success probabilities or evidence that one entire research category dominates another. Methods, meshes, native objectives, collision approximations and finite budgets differ. Some implementations reproduce a selected paper component or replace a numerical backend; their individual source mappings are essential context.

## What the measurements support

**Precise parking exposes the gap between finding a route and executing it to the prescribed pose.** HA+CG finds all 12 native paths, all 12 have zero measured collision frames, and seven meet the execution terminal tolerance. DPGrid passes both checks in nine of its ten solved tasks. BL_Dijkstra reports goal-cell arrival in all 12, but none of those executions meets the common centimetre-level endpoint check; GraphBellman's eight native successes also have no terminal passes. Their finite goal neighborhoods and native objective definitions must be stated, rather than calling these twelve continuous tasks infeasible.

**General geometric constructions remain useful in irregular local scenes.** RTR_TTS passes both checks in all 12; BicchiTangents in eight; LamirauxSmooth in nine, with one evaluator failure. These are obstacle-aware constructions, not only templates for regular parking slots. Their results cannot be generalized to every closed-form parking template. Sinusoid_RTR passes both checks in eleven, but cases 5 and 12 take approximately 728 and 922 seconds of executed maneuvering with 274 and 230 gear changes. Its 1990 source supplies a steering construction, and the RTR obstacle adapter is explicitly separate. Feasibility alone does not mean an efficient parking maneuver.

**Sampling methods show a coverage-versus-executability distinction.** BiRRT_HC finds eleven native paths and all eleven have zero collision frames, while five reach the terminal tolerance. BiRRT_HCR and CC_PRM each solve seven tasks and pass both checks in all seven. KinoDeform passes both checks in all nine solved tasks. These observations motivate checking curvature/control continuity and exact connections alongside search coverage; they do not establish causation from a single comparison or eliminate the effects of budgets and random seeds.

**The numerical methods have the widest spread in this implementation set.** STC passes both checks in all twelve. SE2_NMPC passes in nine; LIOM and DL_IAPS_PJSO each in eight; IndirectOCP in seven. Initialization, body representation, transcription and solver settings materially affect the outcome. The LIOM refinement study is a direct example: changing an algebraically equivalent expression can alter solver sparsity, while an inner time limit must be distinguished from outer-algorithm failure. Its current configuration has exactly three covering discs and 201 states, with all eight solved tasks passing both execution checks. See the [LIOM study](LIOM_REFINEMENT.md).

Several weak outcomes have specific scope mismatches. BOMP's printed formulation has no acceleration bound and uses a 15-node global polynomial, whose between-node overshoots are documented. TriangleArea retains printed dynamics that differ from the common rear-axle model. LatticeOCP and some circle-based methods reject endpoints that the physical rectangle can occupy. Soft penalties and sampled or under-covering collision approximations can leave physical intersections. These measurements do not isolate the merits of pseudospectral methods, triangle-area constraints, lattices or optimization as broad classes.

**STC and RTR_TTS currently have the most complete joint coverage in this set.** Both pass all twelve. On those same twelve tasks, their median execution durations are approximately 13.4 and 24.1 seconds. RTR_TTS needs 18, 24 and 21 gear changes in cases 2, 9 and 12. STC's planner wall-time median is approximately 4.4 seconds, but its case-2 time is 70.8 seconds. These separate dimensions remain preferable to declaring a universal winner.

## Use in a survey

The defensible conclusion is that terminal parking tests exact endpoint delivery, physically executable turning and complete-body clearance simultaneously. A route-search success flag is insufficient, and a smooth native drawing is insufficient. Search and general geometric constructions offer useful route coverage; continuous steering constructions or optimal control can improve execution behavior, subject to their own initialization and representation limits. The strongest results here come from more than one methodological family.

Before turning the descriptive counts into comparative research claims, review the disclosed adaptations, conduct common-hardware repeated timing, repeat stochastic methods, and study mesh/budget/terminal-tolerance sensitivity. Keep the published 1 cm / 1 degree protocol fixed while reporting any sensitivity study separately. Planning time and executed maneuver time are different quantities. Pooled execution/smoothness averages over only successful tasks compare different case subsets; use common solved cases for pairwise quality comparisons. No formal significance, global optimum, author-certified reproduction or category-wide ranking is claimed here.

## All implementations and memberships

Each method has twelve attempted tasks. `Both` means successfully evaluated, terminal attained, and zero collision frames. Median planning time includes failed calls and is descriptive only: concurrent development load, different thread counts and one recorded trial preclude controlled runtime rankings. The latest LIOM run also includes a shared setup-path performance fix; the other 39 recorded timing sets were not rerun after that fix. This makes a direct cross-method runtime ranking especially inappropriate. The other 39 outcome records are unchanged from commit 502421de44b46107603c1d8d77ee463e782e5884.

### Analytic geometry and constructive steering

| Method and source mapping | Planner | Evaluated | Terminal | Zero collision | Both | Median plan (s) |
|---|---:|---:|---:|---:|---:|---:|
| [LaumondRS](../planners/LaumondRS/README.md) | 12 | 12 | 4 | 4 | 1 | 2.964 |
| [LamirauxSmooth](../planners/LamirauxSmooth/README.md) | 12 | 11 | 11 | 9 | 9 | 62.736 |
| [BicchiTangents](../planners/BicchiTangents/README.md) | 10 | 10 | 8 | 10 | 8 | 6.114 |
| [RTR_TTS](../planners/RTR_TTS/README.md) | 12 | 12 | 12 | 12 | 12 | 11.435 |
| [Sinusoid_RTR](../planners/Sinusoid_RTR/README.md) | 12 | 12 | 12 | 11 | 11 | 11.706 |

### Sampling / roadmaps / exploration trees

| Method and source mapping | Planner | Evaluated | Terminal | Zero collision | Both | Median plan (s) |
|---|---:|---:|---:|---:|---:|---:|
| [BiRRT_HC](../planners/BiRRT_HC/README.md) | 11 | 11 | 5 | 11 | 5 | 7.243 |
| [BiRRT_HCR](../planners/BiRRT_HCR/README.md) | 7 | 7 | 7 | 7 | 7 | 9.504 |
| [CC_PRM](../planners/CC_PRM/README.md) | 7 | 7 | 7 | 7 | 7 | 12.639 |
| [CPRM](../planners/CPRM/README.md) | 7 | 7 | 4 | 7 | 4 | 3.890 |
| [WGRRT](../planners/WGRRT/README.md) | 11 | 11 | 6 | 5 | 2 | 7.623 |
| [SmoothBiRRT](../planners/SmoothBiRRT/README.md) | 6 | 6 | 4 | 6 | 4 | 69.752 |
| [KinoDeform](../planners/KinoDeform/README.md) | 9 | 9 | 9 | 9 | 9 | 114.163 |

### Discrete search / value / reachability

| Method and source mapping | Planner | Evaluated | Terminal | Zero collision | Both | Median plan (s) |
|---|---:|---:|---:|---:|---:|---:|
| [HA_CG](../planners/HA_CG/README.md) | 12 | 12 | 7 | 12 | 7 | 3.550 |
| [DPGrid](../planners/DPGrid/README.md) | 10 | 10 | 9 | 10 | 9 | 10.028 |
| [BL_Dijkstra](../planners/BL_Dijkstra/README.md) | 12 | 12 | 0 | 9 | 0 | 41.478 |
| [GraphBellman](../planners/GraphBellman/README.md) | 8 | 8 | 0 | 8 | 0 | 36.487 |
| [OSEHS](../planners/OSEHS/README.md) | 10 | 10 | 6 | 9 | 5 | 12.759 |
| [BIAGT](../planners/BIAGT/README.md) | 8 | 8 | 6 | 2 | 2 | 71.758 |
| [HJBA](../planners/HJBA/README.md) | 12 | 12 | 5 | 10 | 4 | 84.050 |
| [DubinsGrid](../planners/DubinsGrid/README.md) | 11 | 11 | 10 | 5 | 5 | 6.897 |
| [LatticeOCP](../planners/LatticeOCP/README.md) | 2 | 2 | 2 | 2 | 2 | 1.469 |

### Numerical optimization and optimization-dominant hybrids

| Method and source mapping | Planner | Evaluated | Terminal | Zero collision | Both | Median plan (s) |
|---|---:|---:|---:|---:|---:|---:|
| [STC](../planners/STC/README.md) | 12 | 12 | 12 | 12 | 12 | 4.404 |
| [LIOM](../planners/LIOM/README.md) | 8 | 8 | 8 | 8 | 8 | 3.849 |
| [H_OBCA](../planners/H_OBCA/README.md) | 12 | 12 | 4 | 10 | 4 | 2.998 |
| [TDR_OBCA](../planners/TDR_OBCA/README.md) | 12 | 12 | 8 | 7 | 5 | 20.792 |
| [HyperplaneOCP](../planners/HyperplaneOCP/README.md) | 6 | 6 | 6 | 2 | 2 | 13.059 |
| [TriangleArea](../planners/TriangleArea/README.md) | 4 | 4 | 0 | 0 | 0 | 9.390 |
| [BOMP](../planners/BOMP/README.md) | 2 | 2 | 0 | 0 | 0 | 31.796 |
| [SLiFS](../planners/SLiFS/README.md) | 6 | 6 | 6 | 6 | 6 | 2.789 |
| [DL_IAPS_PJSO](../planners/DL_IAPS_PJSO/README.md) | 10 | 10 | 10 | 8 | 8 | 17.811 |
| [Eta3](../planners/Eta3/README.md) | 8 | 8 | 8 | 5 | 5 | 7.906 |
| [TEB](../planners/TEB/README.md) | 11 | 11 | 0 | 4 | 0 | 9.608 |
| [SE2_NMPC](../planners/SE2_NMPC/README.md) | 11 | 11 | 9 | 11 | 9 | 7.835 |
| [VPF](../planners/VPF/README.md) | 3 | 3 | 2 | 3 | 2 | 121.022 |
| [RITP](../planners/RITP/README.md) | 12 | 12 | 5 | 12 | 5 | 28.077 |
| [AnytimePSRO](../planners/AnytimePSRO/README.md) | 10 | 10 | 0 | 4 | 0 | 21.753 |
| [TPCKC](../planners/TPCKC/README.md) | 5 | 5 | 5 | 5 | 5 | 14.593 |
| [DFTPAV_Path](../planners/DFTPAV_Path/README.md) | 11 | 11 | 8 | 8 | 7 | 14.131 |
| [IndirectOCP](../planners/IndirectOCP/README.md) | 7 | 7 | 7 | 7 | 7 | 76.200 |
| [PointPotentialOCP](../planners/PointPotentialOCP/README.md) | 2 | 2 | 0 | 2 | 0 | 53.978 |

## Post-hoc tolerance diagnostic

This reclassifies existing measured terminal errors only; it does not change the official protocol, rerun tracking or replace any result flag. With the heading threshold fixed at 1 degree, the count of zero-collision executions passing both coordinate thresholds is:

| Each-coordinate threshold | Joint count / 480 |
|---|---:|
| 1 cm | 198 |
| 2 cm | 202 |
| 5 cm | 211 |

[All per-case measurements](RESULT_INDEX.md) retain individual times, collision percentages, effort, steering integrals, gear changes, failure codes and native/evaluation flags.
