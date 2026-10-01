function result = Plan(c)
% Dolgov et al., IJRR 2010: Hybrid A* followed by nonlinear conjugate gradient.
options = hacg.Config();
result = cp.EmptyResult('HA_CG',c.id,'path');
raw = parking.SearchHybridAStar(c,options.search);
result.solver.search_success = raw.success;
if ~raw.success
    result.status.code = 'search_failed';
    result.status.message = 'Hybrid A* exhausted its search budget.';
    return;
end
[smoothed,details] = hacg.Smooth(c,raw,options.cg);
cfg = BenchmarkConfig(); t = c.task;
result.path = cp.PathFromArcs([t.x0 t.y0 t.theta0],smoothed.primitives,c.vehicle,cfg.output.path_spacing_max_m);
result.status.success = true;
result.status.code = 'path_found';
result.status.message = 'A path is available; evaluation is independent of planner success.';
result.solver.coarse_cg_exitflag = details.coarse.runs{end}.exitflag;
result.solver.fine_cg_exitflag = details.fine.runs{end}.exitflag;
result.solver.retained_search_path = details.coarse.returned_input && details.fine.returned_input;
result.diagnostics.search = raw;
result.diagnostics.cg = details;
end
