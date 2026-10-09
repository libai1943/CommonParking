function result=Plan(c)
biagt.EnsureNative();o=biagt.Config(c.vehicle);result=cp.EmptyResult('BIAGT',c.id,'path');old=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(old)); %#ok<NASGU>
[arcs,search]=biagt.Search(c,o);result.diagnostics=struct('options',o,'search',search,'primitives',arcs);result.solver=struct('success',~isempty(arcs),'iterations',search.iterations,'nodes',search.nodes);
if isempty(arcs),result.status.code='search_exhausted';result.status.message='BIAGT did not find a continuously collision-free exact terminal attachment within its finite budget.';return;end
start=[c.task.x0 c.task.y0 c.task.theta0];result.path=cp.PathFromArcs(start,arcs,c.vehicle,o.outputSpacing);p=result.path;
error=[p.x(end)-c.task.xf,p.y(end)-c.task.yf,atan2(sin(p.theta(end)-c.task.thetaf),cos(p.theta(end)-c.task.thetaf))];assert(max(abs(error))<1e-7);
result.status=struct('success',true,'code','path_found','message','Prioritized bidirectional A-search and the disclosed local terminal attachment produced a full-body collision-free path.');
end
