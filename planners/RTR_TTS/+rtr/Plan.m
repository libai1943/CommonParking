function result=Plan(c)
ccp.EnsureNative();o=rtr.Config(c.vehicle);prior=rng;rng(o.seed+c.id);restore=onCleanup(@()rng(prior)); %#ok<NASGU>
result=cp.EmptyResult('RTR_TTS',c.id,'path');v=c.vehicle;body=[v.lw+v.lf,v.lr,v.lb/2];ob=parking.PolygonData(c);t=c.task;start=[t.x0,t.y0,t.theta0];goal=[t.xf,t.yf,t.thetaf];
[path,globalInfo]=rtr.Global(start,goal,ob.vertices,body,o);result.diagnostics=struct('options',o,'geometric_path',path,'global_search',globalInfo);result.solver=struct('success',false);
if ~globalInfo.success,result.status.code='global_search_failed';result.status.message='RTR trees were not connected within the finite geometric search budget.';return;end
[u,localInfo]=rtr.Subdivide(path,ob.vertices,body,o);result.diagnostics.local_search=localInfo;result.diagnostics.controls=u;result.diagnostics.start=[start,0];
if ~localInfo.success,result.status.code='subdivision_failed';result.status.message='The continuous-curvature approximation exhausted its finite subdivision budget.';return;end
result.path=ccp.Path([start,0],u,v,o.outputStep);err=[result.path.x(end),result.path.y(end),result.path.theta(end)]-goal;err(3)=atan2(sin(err(3)),cos(err(3)));assert(max(abs(err))<1e-7);
result.solver.success=true;result.status=struct('success',true,'code','solved','message','RTR geometric path approximated by sampled TTS and exact eeS continuous-curvature connections.');
end
