function result=Plan(c)
options=laumond.Config();result=cp.EmptyResult('LaumondRS',c.id,'path');state=rng;restore=onCleanup(@()rng(state));rng(options.seedBase+c.id); %#ok<NASGU>
connection=reedsSheppConnection('MinTurningRadius',c.vehicle.turning_radius_min,'ReverseCost',1);
[geometric,first]=laumond.GeometricPath(c,options);result.solver.geometric=first;
if ~first.success,result.status.code='geometric_search_failed';result.status.message='The bounded holonomic search did not find a path.';return;end
[primitives,second]=laumond.Subdivide(geometric,c,options,connection);result.solver.subdivision=second;
result.diagnostics=struct('geometric_path',geometric,'options',options);
if ~second.success,result.status.code='subdivision_failed';result.status.message='Recursive Reeds-Shepp subdivision exceeded its resource bounds.';return;end
start=[c.task.x0 c.task.y0 c.task.theta0];raw=primitives;
[primitives,third]=laumond.Shorten(start,primitives,c,options,connection);result.solver.shortening=third;
cfg=BenchmarkConfig();result.path=cp.PathFromArcs(start,primitives,c.vehicle,cfg.output.path_spacing_max_m);
endPose=[result.path.x(end),result.path.y(end),result.path.theta(end)];target=[c.task.xf c.task.yf c.task.thetaf];error=endPose-target;error(3)=atan2(sin(error(3)),cos(error(3)));
assert(max(abs(error))<1e-6&&laumond.ArcsFree(start,primitives,c),'The final path failed geometric validation.');
result.status=struct('success',true,'code','path_found','message','Geometric path, recursive shortest-curve subdivision and collision-free random shortening completed.');
result.solver.success=true;result.diagnostics.unshortened_primitives=raw;result.diagnostics.primitives=primitives;
end
