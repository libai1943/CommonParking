function result = Plan(c)
% Dolgov et al., IJRR 2010: Hybrid A* followed by nonlinear conjugate gradient.
options = hacg.Config();
result = cp.EmptyResult('HA_CG',c.id,'path');
stageClock=tic;geometry=hacg.Geometry(c);
raw = parking.SearchHybridAStar(c,options.search,geometry);
timings=struct('search_wall_s',toc(stageClock),'search_loop_s',raw.runtime, ...
    'heuristic_setup_s',raw.heuristic_setup_time,'cg_wall_s',0,'output_wall_s',0);
result.diagnostics.search=raw;result.diagnostics.timings=timings;
result.solver.search_success = raw.success;
if ~raw.success
    result.status.code = 'search_failed';
    result.status.message = 'Hybrid A* exhausted its search budget.';
    return;
end
stageClock=tic;[smoothed,details] = hacg.Smooth(c,raw,options.cg);
timings.cg_wall_s=toc(stageClock);stageClock=tic;
cfg = BenchmarkConfig(); t = c.task;
result.path = cp.PathFromArcs([t.x0 t.y0 t.theta0],smoothed.primitives,c.vehicle,cfg.output.path_spacing_max_m);
timings.output_wall_s=toc(stageClock);
result.status.success = true;
result.status.code = 'path_found';
result.status.message = 'A path is available; evaluation is independent of planner success.';
result.solver.coarse_cg_exitflag = details.coarse.runs{end}.exitflag;
result.solver.fine_cg_exitflag = details.fine.runs{end}.exitflag;
result.solver.retained_search_path = details.coarse.returned_input && details.fine.returned_input;
result.diagnostics.search = raw;
result.diagnostics.cg = details;
result.diagnostics.timings=timings;
end
