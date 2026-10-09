function result=Plan(c)
o=gbellman.Config();result=cp.EmptyResult('GraphBellman',c.id,'path');
folder=getenv('COMMONPARKING_GRAPH_BELLMAN_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','graph-bellman',computer('arch'));end
if isfolder(folder),addpath(folder);end
assert(exist('graph_bellman_mex','file')==3,'Run BuildGraphBellman once before using GraphBellman.');
v=c.vehicle;polygons=parking.PolygonData(c);origin=[c.task.xf c.task.yf];angle=c.task.thetaf;
R=[cos(angle) -sin(angle);sin(angle) cos(angle)];startXY=([c.task.x0 c.task.y0]-origin)*R;
start=[startXY atan2(sin(c.task.theta0-angle),cos(c.task.theta0-angle))];local=polygons.vertices;
points=[startXY;0 0];for j=1:numel(local),local{j}=(local{j}-origin)*R;points=[points;local{j}];end %#ok<AGROW>
bounds=[floor((min(points,[],1)-o.padding)/o.spacing),ceil((max(points,[],1)+o.padding)/o.spacing)]*o.spacing;
parameters=[v.lw+v.lf v.lr v.lb/2 v.kappa_max o.spacing o.headingCount o.yawCount o.stepDuration o.discount o.switchPenalty o.residualTolerance o.maximumSeconds o.maximumPolicySteps];
[arcs,stats,values]=graph_bellman_mex(start,local,bounds,parameters);
result.solver=struct('success',stats(1)==1,'code',stats(1),'state_count',stats(2),'queue_updates',stats(3),'residual',stats(4),'graph_build_time_s',stats(5),'bellman_time_s',stats(6),'policy_steps',stats(7),'policy_value',stats(8));
result.diagnostics=struct('options',o,'local_bounds',bounds,'local_origin',origin,'local_angle',angle,'value_function',values,'primitives',arcs);
if isempty(arcs)
 result.status.code='bellman_or_policy_failed';result.status.message='The graph solve or unmodified greedy feedback rollout failed; no fallback path was substituted.';return;
end
result.path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],arcs,v,o.outputSpacing);
result.status=struct('success',stats(1)==1,'code','goal_neighborhood_reached','message','The Model 1 greedy policy entered the disclosed finite-grid goal neighborhood; obstacle costs are soft and no endpoint repair is applied.');
if stats(1)~=1,result.status.code='policy_failed';result.status.message='The greedy policy did not enter the finite-grid goal neighborhood within its limits; the partial path is retained.';end
end
