function result=Plan(c)
o=psro.Config();result=cp.EmptyResult('AnytimePSRO',c.id,'trajectory');previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
result.diagnostics=struct('options',o);raw=parking.SearchHybridAStar(c,o.search);result.diagnostics.search=raw;
if ~raw.success,result.status.code='initialization_failed';result.status.message='Hybrid A* did not find the paper reference path.';return;end
t=c.task;path=cp.PathFromArcs([t.x0 t.y0 t.theta0],raw.primitives,c.vehicle,.05);history=cell(o.maximumOuterIterations,1);
bestCost=inf;best=[];termination='outer_iteration_limit';attempt=0;
for iteration=1:o.maximumOuterIterations
 attempt=iteration;seed=cp.EmptyResult('PSRO_seed',c.id,'path');seed.path=path;ref=cpe.MakeReference(seed,c);
 reference=cpe.ReferenceAt(ref,linspace(0,ref.tf,o.nodes)',c.vehicle);data=psro.Constraints(reference,c,o);
 if data.minimum_reference_area_excess<=0
  termination='reference_vertex_collision';history{iteration}=struct('constraints',data,'reference',reference);break;
 end
 [q,solver,native]=psro.Solve(c,reference,data,o);history{iteration}=struct('solver',solver,'native',native);
 if ~solver.success
  termination='optimization_failed';if isempty(best),result.solver=solver;end;break;
 end
 gap=psro.PathGap(path,q);history{iteration}.path_gap=gap;
 if solver.objective<bestCost,bestCost=solver.objective;best=struct('q',q,'solver',solver,'native',native,'iteration',iteration);end
 if gap<o.gapTolerance,termination='path_gap_converged';break;end
 path=psro.Repath(q,c.vehicle);
end
result.diagnostics.history=history(1:attempt);
if isempty(best),result.status.code=termination;result.status.message='No accepted trajectory entered the anytime candidate pool.';return;end
result.trajectory=best.q;result.solver=best.solver;result.solver.outer_iterations=attempt;result.solver.selected_iteration=best.iteration;result.solver.termination=termination;
result.diagnostics.native=best.native;result.status=struct('success',true,'code','solved','message','Returned the minimum-cost accepted native trajectory from the anytime pool.');
end
