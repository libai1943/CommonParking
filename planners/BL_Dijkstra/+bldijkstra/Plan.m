function result=Plan(c)
o=bldijkstra.Config();result=cp.EmptyResult('BL_Dijkstra',c.id,'path');
folder=getenv('COMMONPARKING_BL_DIJKSTRA_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','bl-dijkstra',computer('arch'));end
if isfolder(folder),addpath(folder);end
assert(exist('bl_dijkstra_mex','file')==3,'Run BuildBLDijkstra once before using BL_Dijkstra.');
polygons=parking.PolygonData(c);points=[c.task.x0 c.task.y0;c.task.xf c.task.yf];for j=1:polygons.count,points=[points;polygons.vertices{j}];end %#ok<AGROW>
bounds=[min(points,[],1)-o.padding,max(points,[],1)+o.padding];v=c.vehicle;
cellSize=[(bounds(3)-bounds(1))/2^o.resolutionExponent,(bounds(4)-bounds(2))/2^o.resolutionExponent,2*pi/2^o.resolutionExponent];
% Paper controls specify front-axle speed +/-1, not rear-axle speed.
headingBinsPerTurn=ceil(1.1*sum(cellSize(1:2))*v.kappa_max/cellSize(3));
if mod(headingBinsPerTurn,2)==0,headingBinsPerTurn=headingBinsPerTurn+1;end
duration=headingBinsPerTurn*cellSize(3)*v.lw/sin(v.phimax);maximumDepth=2^(3*o.resolutionExponent);
start=[c.task.x0 c.task.y0 c.task.theta0];goal=[c.task.xf c.task.yf c.task.thetaf];
parameters=[v.lw+v.lf v.lr v.lb/2 v.lw v.phimax o.resolutionExponent duration maximumDepth o.maximumSeconds o.maximumNodes];
[primitives,stats]=bl_dijkstra_mex(start,goal,polygons.vertices,bounds,parameters);
result.diagnostics=struct('options',o,'bounds',bounds,'cell_size',cellSize,'front_speed_step_duration',duration,'heading_bins_per_turn',headingBinsPerTurn,'maximum_depth',maximumDepth);
result.solver=struct('success',~isempty(primitives),'expanded',stats(1),'generated',stats(2),'collision_checks',stats(3),'search_time_s',stats(4),'budget_exhausted',logical(stats(5)),'reversals',stats(6),'path_length_m',stats(7));
if isempty(primitives),result.status.code='search_failed';result.status.message='No goal-cell route was selected before exhausting the finite indexed search or its resource limits.';return;end
result.path=cp.PathFromArcs(start,primitives,v,o.outputSpacing);result.diagnostics.primitives=primitives;
result.status=struct('success',true,'code','goal_cell_reached','message','The selected path reaches the goal grid cell, as in Section 6.3; the endpoint is not snapped to the exact task goal.');
end
