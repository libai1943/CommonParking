function result = Plan(c)
o=tdr.Config();result=cp.EmptyResult('TDR_OBCA',c.id,'trajectory');
raw=parking.SearchHybridAStar(c,o.search);result.solver.search_success=raw.success;
result.diagnostics=struct('search',raw,'options',o);
if ~raw.success,result.status.code='search_failed';result.status.message='Hybrid A* did not connect the task.';return;end
[initial,temporal]=tdr.Initialize(c,raw.primitives,o);result.diagnostics.temporal=temporal;
if ~temporal.success,result.status.code='temporal_QP_failed';result.status.message='The temporal warm-start QP failed.';return;end
[lambda,mu,d,dual]=tdr.DualSeed(c,[initial.x,initial.y,initial.theta],parking.PolygonData(c),o);
[q,solver,native]=tdr.Solve(c,initial,lambda,mu,d,o);result.solver=solver;result.solver.search_success=true;
result.diagnostics.initial=initial;result.diagnostics.dual_seed=dual;result.diagnostics.native=native;
if ~isempty(q),result.trajectory=q;end
if solver.success
 result.status=struct('success',true,'code','solved','message','TDR-OBCA printed objective and fixed-time NLP converged; terminal pose is soft.');
else
 result.status.code='optimization_failed';result.status.message=solver.message;
end
end
