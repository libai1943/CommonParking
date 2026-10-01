function result = RunPlanner(plannerName,caseId)
%RUNPLANNER Solve exactly one case, with settings owned by the selected planner.
% result = RunPlanner('HA_CG',1)
arguments
    plannerName (1,1) string
    caseId (1,1) double {mustBeInteger,mustBeInRange(caseId,1,12)}
end
started = tic;
SetupCommonParking();
caseData = LoadCase(caseId);
result = cp.SolveCase(plannerName,caseData);
result.computation_time_s = toc(started);
end
