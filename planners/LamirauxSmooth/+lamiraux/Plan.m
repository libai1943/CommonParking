function result=Plan(c)
o=lamiraux.Config();result=cp.EmptyResult('LamirauxSmooth',c.id,'path');state=rng;restore=onCleanup(@()rng(state));rng(o.seedBase+c.id); %#ok<NASGU>
[route,first]=lamiraux.GeometricPath(c,o);result.solver.geometric=first;
if ~first.success,result.status.code='geometric_search_failed';result.status.message='The bounded holonomic planner did not find a route.';return;end
[pieces,second]=lamiraux.Subdivide(route,c,o);result.solver.subdivision=second;result.diagnostics=struct('geometric_path',route,'options',o);
if ~second.success,result.status.code='subdivision_failed';result.status.message='Smooth admissible subdivision exhausted its finite resource budget.';return;end
raw=pieces;[pieces,third]=lamiraux.Shorten(pieces,c,o);result.solver.shortening=third;
result.path=lamiraux.Export(pieces,c.vehicle,o.outputSpacing);last=[result.path.x(end),result.path.y(end),result.path.theta(end)];error=last-[c.task.xf c.task.yf c.task.thetaf];error(3)=atan2(sin(error(3)),cos(error(3)));
assert(max(abs(error))<1e-6,'The composed path misses the goal.');
for j=1:numel(pieces),assert(lamiraux.Certify(pieces(j),c,o,true),'Final continuous local-curve validation failed.');end
result.status=struct('success',true,'code','path_found','message','Smooth canonical-curve approximation and shortening completed.');
result.solver.success=true;result.diagnostics.pieces=pieces;result.diagnostics.unshortened_pieces=raw;
end
