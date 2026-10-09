function result=Plan(c)
ccp.EnsureNative();v=c.vehicle;o=sinsteer.Config(v);prior=rng;rng(o.seed+c.id);restore=onCleanup(@()rng(prior)); %#ok<NASGU>
result=cp.EmptyResult('Sinusoid_RTR',c.id,'path');body=[v.lw+v.lf,v.lr,v.lb/2];ob=parking.PolygonData(c);t=c.task;start=[t.x0,t.y0,t.theta0];goal=[t.xf,t.yf,t.thetaf];
[geometric,globalInfo]=rtr.Global(start,goal,ob.vertices,body,o);result.diagnostics=struct('options',o,'geometric_path',geometric,'global_search',globalInfo);result.solver=struct('success',false);
if ~globalInfo.success,result.status.code='global_search_failed';result.status.message='The disclosed RTR adapter did not connect its geometric trees.';return;end
[phases,localInfo]=sinsteer.Subdivide(geometric,v,ob.vertices,body,o);result.diagnostics.local_search=localInfo;result.diagnostics.phases=phases;
if ~localInfo.success,result.status.code='subdivision_failed';result.status.message='The sinusoidal connection adapter exhausted its finite subdivision budget.';return;end
result.path=sinsteer.Path(phases,o.outputStep);error=[result.path.x(end),result.path.y(end),result.path.theta(end)]-goal;error(3)=atan2(sin(error(3)),cos(error(3)));assert(max(abs(error))<1e-7);
result.solver.success=true;result.status=struct('success',true,'code','solved','message','Three-stage nonlinear sinusoidal steering with an explicitly separate RTR obstacle-planning adapter.');
end
