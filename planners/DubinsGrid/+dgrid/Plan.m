function result=Plan(c)
o=dgrid.Config();result=cp.EmptyResult('DubinsGrid',c.id,'path');
folder=getenv('COMMONPARKING_DUBINS_GRID_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','dubins_grid',computer('arch'));end
if isfolder(folder),addpath(folder);end
assert(exist('dubins_grid_mex','file')==3,'Run BuildDubinsGrid once before using DubinsGrid.');
v=c.vehicle;t=c.task;start=[t.x0 t.y0 t.theta0];goal=[t.xf t.yf t.thetaf];polygons=parking.PolygonData(c);points=[start(1:2);goal(1:2);vertcat(polygons.vertices{:})];
bounds=[floor((min(points)-o.padding)/o.spacing)*o.spacing;ceil((max(points)+o.padding)/o.spacing)*o.spacing];
[x,y,theta]=ndgrid(bounds(1,1):o.spacing:bounds(2,1),bounds(1,2):o.spacing:bounds(2,2),(0:o.headings-1)*2*pi/o.headings);
poses=[start;goal;x(:),y(:),theta(:)];centres=-v.lr+((1:3)-.5)*v.length/3;
result.diagnostics=struct('options',o,'bounds',bounds,'disc_centres',centres,'disc_radius',v.lb/2,'node_count',size(poses,1));
if any(dgrid.DiscGap([start;goal],polygons.vertices,centres,v.lb/2,bounds)<=0,'all')
 result.status.code='endpoint_disc_collision';result.status.message='The paper three-disc model blocks a required task endpoint.';return;
end
[edgePrimitives,stats,route]=dubins_grid_mex('search',poses,polygons.vertices,centres,v.lb/2,bounds,[v.kappa_max,o.edgeLength,o.seconds,o.collisionSpacing]);pp=edgePrimitives(abs(edgePrimitives(:,1))>1e-10,:);
result.solver=struct('success',logical(stats(1)),'expanded',stats(2),'candidate_pairs',stats(3),'collision_checks',stats(4),'search_time_s',stats(5),'budget_exhausted',logical(stats(6)),'free_nodes',stats(7),'reopened',stats(8),'search_length_m',stats(9));
result.diagnostics.search_primitives=pp;result.diagnostics.grid_route=route;result.diagnostics.grid_edge_primitives=edgePrimitives;
if ~stats(1),result.status.code='search_failed';result.status.message='No Dubins-grid path was found within the finite grid and search budget.';return;end
raw=cp.PathFromArcs(start,pp,v,o.outputSpacing);result.diagnostics.raw_path=raw;
[result.path,smoothing,detail]=dgrid.Smooth(raw,c,centres,bounds,o);result.solver.smoothing=smoothing;result.diagnostics.smoothing=detail;
if smoothing.accepted,code='smoothed_path';message='Dubins-grid search followed by a successful sampled Frenet-offset SQP.';else,code='search_path_retained';message='The native smoother did not pass its flag/constraints; the paper permits retaining the feasible unsmoothed search path.';end
result.status=struct('success',true,'code',code,'message',message);
end
