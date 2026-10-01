function result = SolveCase(name,c)
% Dispatch only released methods. Native exceptions become explicit failures.
name = upper(string(name));
switch name
    case {"HA_CG","HA+CG"}
        canonical = 'HA_CG'; planner = @hacg.Plan;kind='path';
    case "STC"
        canonical = 'STC'; planner = @stc.Plan;kind='trajectory';
    case "TRIANGLEAREA"
        canonical = 'TriangleArea'; planner = @triangle.Plan;kind='trajectory';
    case {"H_OBCA","OBCA"}
        canonical = 'H_OBCA'; planner = @hobca.Plan;kind='trajectory';
    otherwise
        error('CommonParking:UnknownPlanner','Unknown released planner: %s',name);
end
result = cp.EmptyResult(canonical,c.id,kind);
try
    result = planner(c);
catch problem
    result.status.message = problem.message;
    result.status.code = 'planner_exception';
    result.diagnostics.exception = getReport(problem,'extended','hyperlinks','off');
end
end
