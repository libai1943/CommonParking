function result=Plan(c)
o=osehs.Config();result=cp.EmptyResult('OSEHS',c.id,'path');[route,exploration]=osehs.Explore(c,o);result.solver.exploration=exploration;
result.diagnostics=struct('options',o,'circle_path',route);
if ~exploration.success,result.status.code=exploration.code;result.status.message='Orientation-aware circle exploration found no route within its finite limits.';return;end
clock=tic;attempts=cell(numel(o.stepFactors),1);pp=zeros(0,2);
for j=1:numel(o.stepFactors)
 remaining=o.searchSeconds-toc(clock);if remaining<=0,break;end
 [trial,attempts{j}]=osehs.Search(route,c,o,o.stepFactors(j),remaining/(numel(o.stepFactors)-j+1));
 if attempts{j}.success,pp=trial;break;end
end
result.solver.search_attempts=attempts;
if isempty(pp),result.status.code='kinematic_search_failed';result.status.message='Circle-guided search exhausted its configured step refinements or resource bounds.';return;end
start=[c.task.x0 c.task.y0 c.task.theta0];assert(osehs.ArcsFree(start,pp,c));
objective=osehs.PathCost(start,pp,route,o,c.vehicle.kappa_max);assert(abs(objective-attempts{j}.objective)<1e-7*(1+objective),'Extracted path cost disagrees with the search labels.');
result.solver.recomputed_objective=objective;result.path=cp.PathFromArcs(start,pp,c.vehicle,o.outputSpacing);
error=[result.path.x(end)-c.task.xf,result.path.y(end)-c.task.yf,result.path.theta(end)-c.task.thetaf];error(3)=atan2(sin(error(3)),cos(error(3)));assert(max(abs(error))<1e-6);
result.diagnostics.primitives=pp;result.solver.success=true;result.status=struct('success',true,'code','path_found','message','Directed-circle exploration and circle-guided heuristic search reached the task pose.');
end
