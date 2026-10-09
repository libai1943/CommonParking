function result=Plan(c)
o=wgrrt.Config();result=cp.EmptyResult('WGRRT',c.id,'path');state=rng;restore=onCleanup(@()rng(state));rng(o.seedBase+c.id); %#ok<NASGU>
[W,geometric]=wgrrt.Waypoints(c,o);result.solver.geometric=geometric;result.diagnostics=struct('options',o,'waypoints',W);
if ~geometric.success,result.status.code='geometric_search_failed';result.status.message='Geometric RRT did not find the waypoint chain within its budget.';return;end
connection=reedsSheppConnection('MinTurningRadius',1/c.vehicle.kappa_max,'ForwardCost',1,'ReverseCost',1);
[graph,search]=wgrrt.Search(W,c,o,connection);result.solver.search=search;result.diagnostics.graph=graph;
if ~search.success,result.status.code=search.code;result.status.message='Waypoint-guided kinematic bi-RRT did not connect every waypoint.';return;end
[pp,value]=wgrrt.ValuePath(graph);result.solver.value_iteration=value;result.diagnostics.unshortened_primitives=pp;
start=[c.task.x0 c.task.y0 c.task.theta0];[pp,shortening]=wgrrt.Shorten(start,pp,c,o,connection);result.solver.shortening=shortening;
assert(wgrrt.ArcsFree(start,pp,c),'Final exact swept-body verification failed.');
result.path=cp.PathFromArcs(start,pp,c.vehicle,o.outputSpacing);result.diagnostics.primitives=pp;
error=[result.path.x(end)-c.task.xf,result.path.y(end)-c.task.yf,result.path.theta(end)-c.task.thetaf];error(3)=atan2(sin(error(3)),cos(error(3)));
assert(max(abs(error))<1e-6,'The composed graph path misses the goal.');
result.solver.success=true;result.status=struct('success',true,'code','path_found','message','Waypoint-guided bi-RRT, value iteration and exact collision-checked RS shortening completed.');
end
