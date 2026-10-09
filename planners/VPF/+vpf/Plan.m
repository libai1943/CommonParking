function result=Plan(c)
o=vpf.Config();result=cp.EmptyResult('VPF',c.id,'trajectory');previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
result.diagnostics=struct('options',o);raw=parking.SearchHybridAStar(c,o.search);result.diagnostics.search=raw;
if ~raw.success,result.status.code='initialization_failed';result.status.message='The paper Hybrid A* initialization did not find a path.';return;end
t=c.task;seed=cp.EmptyResult('VPF_seed',c.id,'path');seed.path=cp.PathFromArcs([t.x0 t.y0 t.theta0],raw.primitives,c.vehicle,.05);
ref=cpe.MakeReference(seed,c);initial=cpe.ReferenceAt(ref,linspace(0,ref.tf,o.intervals+1)',c.vehicle);
result.diagnostics.initial=initial;alpha=1;history=cell(o.maximumOuterIterations,1);
for iteration=1:o.maximumOuterIterations
 [q,solver,native]=vpf.Solve(c,initial,alpha,o);result.solver=solver;result.diagnostics.native=native;
 history{iteration}=struct('solver',solver);result.diagnostics.history=history;
 if ~solver.success,result.status.code='optimization_failed';result.status.message=solver.message;return;end
 [required,cover]=vpf.ProtectionAlpha(native.states,native.controls,native.h,c.vehicle,o);history{iteration}.required_alpha=required;history{iteration}.coverage=cover;
 result.diagnostics.history=history;result.solver.outer_iterations=iteration;result.solver.required_alpha=required;
 if ~cover.success,result.solver.success=false;result.status.code=cover.code;result.status.message='The paper frame-enlargement calculation could not certify its algebraic cover condition.';return;end
 if required-alpha<=o.alphaTolerance
  result.trajectory=q;result.solver.coverage=cover;result.status=struct('success',true,'code','solved','message','RK4 NLP converged and the paper protection-frame enlargement iteration reached its tolerance.');return;
 end
 alpha=required;initial=q;
end
result.solver.success=false;result.status.code='enlargement_iteration_limit';result.status.message='The protection-frame enlargement loop exhausted its finite iteration limit.';
end
