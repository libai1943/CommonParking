function result=Plan(c)
o=triangle.Config();result=cp.EmptyResult('TriangleArea',c.id,'trajectory');
raw=parking.SearchHybridAStar(c,o.search);result.solver.search_success=raw.success;
result.diagnostics=struct('search',raw,'options',o);
if ~raw.success,result.status.code='search_failed';result.status.message='Initialization search failed.';return;end
initial=triangle.StoppedSteeringSeed(c,raw,o.nodes);
[trajectory,solver,native]=triangle.Solve(c,initial,o);
result.solver=solver;result.solver.search_success=true;
if ~isempty(trajectory),result.trajectory=trajectory;end
result.diagnostics.native=native;
if solver.success,result.status=struct('success',true,'code','solved','message',solver.message);
else,result.status.code='optimization_failed';result.status.message=solver.message;end
end
