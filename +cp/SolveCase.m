function result = SolveCase(name,c)
% Dispatch only released methods. Native exceptions become explicit failures.
name = upper(string(name));
switch name
    case {"HA_CG","HA+CG"}
        canonical = 'HA_CG'; planner = @hacg.Plan;
    otherwise
        error('CommonParking:UnknownPlanner','Unknown released planner: %s',name);
end
result = cp.EmptyResult(canonical,c.id,'path');
try
    result = planner(c);
catch problem
    result.status.message = problem.message;
    result.status.code = 'planner_exception';
    result.diagnostics.exception = getReport(problem,'extended','hyperlinks','off');
end
end
