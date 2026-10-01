% CommonParking: one scene, one planner, one result, one evaluation.
% Change only these two selectors. Planner settings live in its Config.m.
plannerName = 'HA_CG';
caseId = 1;

% 1. Load the scene explicitly, for inspection and subsequent evaluation.
SetupCommonParking();
caseData = LoadCase(caseId);

% 2. Compute one path or trajectory. The public entry reloads the named case
%    itself, so it also works independently in a fresh MATLAB session.
result = RunPlanner(plannerName,caseId);

% 3. Evaluate the result. Evaluation runtime is excluded from planner time.
evaluation = EvaluateResult(result,caseData);
disp(evaluation.metrics);

% 4. Optional diagnostics: compare reference and tracked execution.
PlotEvaluation(result,evaluation,caseData);

% Explicit persistence, if desired (choose your own output location):
% save(fullfile(tempdir,'CommonParking_example.mat'),'result','evaluation');
