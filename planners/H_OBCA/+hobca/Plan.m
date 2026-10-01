function result = Plan(c)
options=hobca.Config();result=cp.EmptyResult('H_OBCA',c.id,'trajectory');
raw=parking.SearchHybridAStar(c,options.search);result.solver.search_success=raw.success;
result.diagnostics=struct('search',raw,'options',options);
if ~raw.success,result.status.code='search_failed';result.status.message='Hybrid A* did not connect the task.';return;end
seed=cp.EmptyResult('H_OBCA_initial_path',c.id,'path');
seed.path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],raw.primitives,c.vehicle,.05);
reference=cpe.MakeReference(seed,c);
initial=cpe.ReferenceAt(reference,linspace(0,reference.tf,options.nodes)',c.vehicle);
scale=options.referenceTimeMultiplier;initial.t=initial.t*scale;initial.v=initial.v/scale;initial.a=initial.a/scale^2;
polygons=parking.PolygonData(c);
[lambda,mu,distance]=hobca.DualSeed(c,[initial.x initial.y initial.theta],polygons);
[trajectory,solver]=hobca.Solve(c,initial,polygons,lambda,mu,options);
result.solver=solver;result.solver.search_success=true;
if ~isempty(trajectory),result.trajectory=trajectory;end
if solver.success,result.status=struct('success',true,'code','solved','message',solver.message);
else,result.status.code='optimization_failed';result.status.message=solver.message;end
result.diagnostics=struct('search',raw,'initial',initial,'initial_clearances',distance,'options',options);
end
