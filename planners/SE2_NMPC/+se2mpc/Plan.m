function result=Plan(c)
o=se2mpc.Config();result=cp.EmptyResult('SE2_NMPC',c.id,'trajectory');previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
polygons=parking.PolygonData(c);task=c.task;
[~,~,endpointDistance]=se2mpc.DualSeed(c,[task.x0 task.y0 task.theta0;task.xf task.yf task.thetaf],polygons);
result.diagnostics=struct('options',o,'endpoint_clearance',endpointDistance);
if any(endpointDistance<o.clearance-o.feasibilityTolerance,'all')
 result.status.code='endpoint_clearance_blocked';result.status.message='The retained 0.2 m paper safety gap is violated at a required endpoint.';return;
end
raw=parking.SearchHybridAStar(c,o.search);result.diagnostics.search=raw;
if ~raw.success,result.status.code='initialization_failed';result.status.message='The disclosed Hybrid A* seed failed.';return;end
seed=cp.EmptyResult('SE2_NMPC_seed',c.id,'path');seed.path=cp.PathFromArcs([task.x0 task.y0 task.theta0],raw.primitives,c.vehicle,.05);
ref=cpe.MakeReference(seed,c);initial=cpe.ReferenceAt(ref,linspace(0,ref.tf,o.intervals+1)',c.vehicle);
[lambda,mu]=se2mpc.DualSeed(c,[initial.x initial.y initial.theta],polygons);
[q,solver,details]=se2mpc.Solve(c,initial,polygons,lambda,mu,o);result.solver=solver;
result.diagnostics.initial=initial;result.diagnostics.native=details;
if solver.success
 result.trajectory=q;result.status=struct('success',true,'code','solved','message','The wrapped-angle Crank-Nicolson NLP passed native convergence and independent discrete feasibility.');
else,result.status.code='optimization_failed';result.status.message=solver.message;
end
end
