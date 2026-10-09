# Result index

The release contains 40 named implementations tested on the same 12 frozen terminal-parking tasks: 480 recorded planner calls and their evaluation outcomes. The table is alphabetical, not a ranking. Each count has denominator 12. Planner success, successful evaluation, terminal attainment and zero measured collision frames are different properties; the last two columns count only successfully evaluated executions. An entry can reach the terminal tolerance and still collide.

Every method links to its equations, source paper, initialization, numerical settings, dependencies and disclosed adaptations. These are independent implementations, sometimes with explicit search or geometry adapters, solver replacements, or a selected paper component. They are not author-certified reproductions. A failed finite-budget run does not prove that the paper's method or the scene is infeasible. One recorded run per task is provided; seeded sampling plus wall-time limits and concurrent development load do not constitute a statistical success-rate study or a controlled runtime comparison.

The detailed per-method tables in the root README retain all dimensions, failure codes and unavailable values. No overall score is computed. The [evaluation protocol](../evaluation/README.md) defines the ideal-model execution surrogate and its numerical limits.

| Implementation and source mapping | Native success | Evaluated | Terminal attained | Zero collision frames | Twelve-case data |
|---|---:|---:|---:|---:|---|
| [AnytimePSRO](../planners/AnytimePSRO/README.md) | 10 | 10 | 0 | 4 | [JSON](../results/AnytimePSRO/metrics.json) · [CSV](../results/AnytimePSRO/metrics.csv) |
| [BIAGT](../planners/BIAGT/README.md) | 8 | 8 | 6 | 2 | [JSON](../results/BIAGT/metrics.json) · [CSV](../results/BIAGT/metrics.csv) |
| [BicchiTangents](../planners/BicchiTangents/README.md) | 10 | 10 | 8 | 10 | [JSON](../results/BicchiTangents/metrics.json) · [CSV](../results/BicchiTangents/metrics.csv) |
| [BiRRT_HC](../planners/BiRRT_HC/README.md) | 11 | 11 | 5 | 11 | [JSON](../results/BiRRT_HC/metrics.json) · [CSV](../results/BiRRT_HC/metrics.csv) |
| [BiRRT_HCR](../planners/BiRRT_HCR/README.md) | 7 | 7 | 7 | 7 | [JSON](../results/BiRRT_HCR/metrics.json) · [CSV](../results/BiRRT_HCR/metrics.csv) |
| [BL_Dijkstra](../planners/BL_Dijkstra/README.md) | 12 | 12 | 0 | 9 | [JSON](../results/BL_Dijkstra/metrics.json) · [CSV](../results/BL_Dijkstra/metrics.csv) |
| [BOMP](../planners/BOMP/README.md) | 2 | 2 | 0 | 0 | [JSON](../results/BOMP/metrics.json) · [CSV](../results/BOMP/metrics.csv) |
| [CC_PRM](../planners/CC_PRM/README.md) | 7 | 7 | 7 | 7 | [JSON](../results/CC_PRM/metrics.json) · [CSV](../results/CC_PRM/metrics.csv) |
| [CPRM](../planners/CPRM/README.md) | 7 | 7 | 4 | 7 | [JSON](../results/CPRM/metrics.json) · [CSV](../results/CPRM/metrics.csv) |
| [DFTPAV_Path](../planners/DFTPAV_Path/README.md) | 11 | 11 | 8 | 8 | [JSON](../results/DFTPAV_Path/metrics.json) · [CSV](../results/DFTPAV_Path/metrics.csv) |
| [DL_IAPS_PJSO](../planners/DL_IAPS_PJSO/README.md) | 10 | 10 | 10 | 8 | [JSON](../results/DL_IAPS_PJSO/metrics.json) · [CSV](../results/DL_IAPS_PJSO/metrics.csv) |
| [DPGrid](../planners/DPGrid/README.md) | 10 | 10 | 9 | 10 | [JSON](../results/DPGrid/metrics.json) · [CSV](../results/DPGrid/metrics.csv) |
| [DubinsGrid](../planners/DubinsGrid/README.md) | 11 | 11 | 10 | 5 | [JSON](../results/DubinsGrid/metrics.json) · [CSV](../results/DubinsGrid/metrics.csv) |
| [Eta3](../planners/Eta3/README.md) | 8 | 8 | 8 | 5 | [JSON](../results/Eta3/metrics.json) · [CSV](../results/Eta3/metrics.csv) |
| [GraphBellman](../planners/GraphBellman/README.md) | 8 | 8 | 0 | 8 | [JSON](../results/GraphBellman/metrics.json) · [CSV](../results/GraphBellman/metrics.csv) |
| [H_OBCA](../planners/H_OBCA/README.md) | 12 | 12 | 4 | 10 | [JSON](../results/H_OBCA/metrics.json) · [CSV](../results/H_OBCA/metrics.csv) |
| [HA_CG](../planners/HA_CG/README.md) | 12 | 12 | 7 | 12 | [JSON](../results/HA_CG/metrics.json) · [CSV](../results/HA_CG/metrics.csv) |
| [HJBA](../planners/HJBA/README.md) | 12 | 12 | 5 | 10 | [JSON](../results/HJBA/metrics.json) · [CSV](../results/HJBA/metrics.csv) |
| [HyperplaneOCP](../planners/HyperplaneOCP/README.md) | 6 | 6 | 6 | 2 | [JSON](../results/HyperplaneOCP/metrics.json) · [CSV](../results/HyperplaneOCP/metrics.csv) |
| [IndirectOCP](../planners/IndirectOCP/README.md) | 7 | 7 | 7 | 7 | [JSON](../results/IndirectOCP/metrics.json) · [CSV](../results/IndirectOCP/metrics.csv) |
| [KinoDeform](../planners/KinoDeform/README.md) | 9 | 9 | 9 | 9 | [JSON](../results/KinoDeform/metrics.json) · [CSV](../results/KinoDeform/metrics.csv) |
| [LamirauxSmooth](../planners/LamirauxSmooth/README.md) | 12 | 11 | 11 | 9 | [JSON](../results/LamirauxSmooth/metrics.json) · [CSV](../results/LamirauxSmooth/metrics.csv) |
| [LatticeOCP](../planners/LatticeOCP/README.md) | 2 | 2 | 2 | 2 | [JSON](../results/LatticeOCP/metrics.json) · [CSV](../results/LatticeOCP/metrics.csv) |
| [LaumondRS](../planners/LaumondRS/README.md) | 12 | 12 | 4 | 4 | [JSON](../results/LaumondRS/metrics.json) · [CSV](../results/LaumondRS/metrics.csv) |
| [LIOM](../planners/LIOM/README.md) | 8 | 8 | 8 | 8 | [JSON](../results/LIOM/metrics.json) · [CSV](../results/LIOM/metrics.csv) |
| [OSEHS](../planners/OSEHS/README.md) | 10 | 10 | 6 | 9 | [JSON](../results/OSEHS/metrics.json) · [CSV](../results/OSEHS/metrics.csv) |
| [PointPotentialOCP](../planners/PointPotentialOCP/README.md) | 2 | 2 | 0 | 2 | [JSON](../results/PointPotentialOCP/metrics.json) · [CSV](../results/PointPotentialOCP/metrics.csv) |
| [RITP](../planners/RITP/README.md) | 12 | 12 | 5 | 12 | [JSON](../results/RITP/metrics.json) · [CSV](../results/RITP/metrics.csv) |
| [RTR_TTS](../planners/RTR_TTS/README.md) | 12 | 12 | 12 | 12 | [JSON](../results/RTR_TTS/metrics.json) · [CSV](../results/RTR_TTS/metrics.csv) |
| [SE2_NMPC](../planners/SE2_NMPC/README.md) | 11 | 11 | 9 | 11 | [JSON](../results/SE2_NMPC/metrics.json) · [CSV](../results/SE2_NMPC/metrics.csv) |
| [Sinusoid_RTR](../planners/Sinusoid_RTR/README.md) | 12 | 12 | 12 | 11 | [JSON](../results/Sinusoid_RTR/metrics.json) · [CSV](../results/Sinusoid_RTR/metrics.csv) |
| [SLiFS](../planners/SLiFS/README.md) | 10 | 10 | 9 | 7 | [JSON](../results/SLiFS/metrics.json) · [CSV](../results/SLiFS/metrics.csv) |
| [SmoothBiRRT](../planners/SmoothBiRRT/README.md) | 6 | 6 | 4 | 6 | [JSON](../results/SmoothBiRRT/metrics.json) · [CSV](../results/SmoothBiRRT/metrics.csv) |
| [STC](../planners/STC/README.md) | 12 | 12 | 12 | 12 | [JSON](../results/STC/metrics.json) · [CSV](../results/STC/metrics.csv) |
| [TDR_OBCA](../planners/TDR_OBCA/README.md) | 12 | 12 | 8 | 7 | [JSON](../results/TDR_OBCA/metrics.json) · [CSV](../results/TDR_OBCA/metrics.csv) |
| [TEB](../planners/TEB/README.md) | 11 | 11 | 0 | 4 | [JSON](../results/TEB/metrics.json) · [CSV](../results/TEB/metrics.csv) |
| [TPCKC](../planners/TPCKC/README.md) | 5 | 5 | 5 | 5 | [JSON](../results/TPCKC/metrics.json) · [CSV](../results/TPCKC/metrics.csv) |
| [TriangleArea](../planners/TriangleArea/README.md) | 11 | 11 | 9 | 4 | [JSON](../results/TriangleArea/metrics.json) · [CSV](../results/TriangleArea/metrics.csv) |
| [VPF](../planners/VPF/README.md) | 3 | 3 | 2 | 3 | [JSON](../results/VPF/metrics.json) · [CSV](../results/VPF/metrics.csv) |
| [WGRRT](../planners/WGRRT/README.md) | 11 | 11 | 6 | 5 | [JSON](../results/WGRRT/metrics.json) · [CSV](../results/WGRRT/metrics.csv) |

## Reading and checking saved results

```matlab
SetupCommonParking;
[result,metrics,evaluationStatus] = LoadPublishedResult('HA_CG',3);
% To regenerate the execution trace and its optional plots:
evaluation = EvaluateResult(result,LoadCase(3));
PlotEvaluation(result,evaluation,LoadCase(3));
% To inspect all saved records without rerunning planners or tracking:
addpath('tests');
report = AuditPublishedResults();
```

The [artifact audit](ARTIFACT_AUDIT.json) records the verified source snapshot and checks. The [SHA-256 manifest](RESULT_MANIFEST.json) covers saved MAT/chunk files, metric tables and method validation reports. It verifies transport integrity, not algorithmic correctness. The legacy [core validation report](validation.json) covers the result contract, path timing, CG gradients and tracker fixtures; each method also has its own validation report where provided.

The [three-row, four-column HA+CG vector figure](../results/HA_CG/HA_CG_overview.pdf) shows submitted paths and translucent footprints. It is not a plot of the evaluator's tracked executions; its retained search-path fallbacks are documented in the HA+CG result section.

To regenerate a three-row, four-column figure without solving again:

```matlab
results = cell(1,12);
for caseId = 1:12
    results{caseId} = LoadPublishedResult('HA_CG',caseId);
end
fig = PlotOverview(results);
exportgraphics(fig,fullfile(tempdir,'HA_CG_overview.pdf'),'ContentType','vector');
set(fig,'Visible','on');
```

[Timing scope and comparability](ENVIRONMENT.json) accompany the measurements.

[Mechanism-level assessment of all forty methods](METHOD_CATEGORIES.md) distinguishes recorded outcomes from broader research claims.
