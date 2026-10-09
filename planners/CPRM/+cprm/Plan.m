function result=Plan(c)
o=cprm.Config();result=cp.EmptyResult('CPRM',c.id,'path');previous=rng;restore=onCleanup(@()rng(previous));rng(o.seedBase+c.id); %#ok<NASGU>
map=cprm.BuildRoadmap(c,o);result.solver.preprocessing=struct('control_points',size(map.control_points,1),'control_edges',size(map.control_edges,1),'configuration_nodes',size(map.nodes,1),'approximate_edges',size(map.ends,1),'time_s',map.time_s);
[route,query]=cprm.Query(map,c,o);result.solver.query=query;
if ~query.success,result.status.code='roadmap_query_failed';result.status.message='The finite roadmap query did not find a validated path.';return;end
[pieces,smooth]=cprm.Smooth(route,c,o);result.solver.smoothing=smooth;result.path=cprm.Export(pieces,c.vehicle,o.outputSpacing);
for j=1:numel(pieces)
 p=pieces(j);if strcmp(p.type,'arc'),assert(cprm.ArcsFree(p.start,p.primitive,c));else,assert(cprm.CubicFree(p,c,o));end
end
q=[result.path.x([1,end]),result.path.y([1,end]),result.path.theta([1,end])];target=[c.task.x0 c.task.y0 c.task.theta0;c.task.xf c.task.yf c.task.thetaf];error=q-target;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));assert(max(abs(error),[],'all')<1e-6);
result.status=struct('success',true,'code','path_found','message','Customized roadmap query and cubic-spline smoothing/fallback completed.');result.solver.success=true;
result.diagnostics=struct('pieces',pieces,'route',route,'options',o);
end
