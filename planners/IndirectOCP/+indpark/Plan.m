function result=Plan(c)
o=indpark.Config();result=cp.EmptyResult('IndirectOCP',c.id,'trajectory');[z,ctx,raw]=indpark.Initialize(c,o);result.diagnostics=struct('options',o,'search',raw);result.solver=struct('success',false,'stages',{{}});
if ~raw.success,result.status.code='initial_search_failed';result.status.message='Hybrid A* did not produce an initial path.';return;end
[z,info]=indpark.Solve(z,ctx);result.solver.stages{end+1}=info;success=info.success;code='tracking_ocp_failed';
if success
 [z,ctx,records,success]=indpark.ObstacleContinuation(z,ctx);result.solver.stages=[result.solver.stages,records];code='obstacle_continuation_failed';
end
if success
 [z,ctx,records,success]=indpark.Regularize(z,ctx);result.solver.stages=[result.solver.stages,records];code='regularization_failed';
end
[result.trajectory,result.diagnostics.native]=indpark.Output(z,ctx);result.diagnostics.final_regularization=ctx.mu;
if isfield(ctx,'blend'),result.diagnostics.final_obstacle_blend=ctx.blend;else,result.diagnostics.final_obstacle_blend=0;end
if ~success,result.status.code=code;result.status.message='The canonical boundary-value equations did not converge through every required stage.';return;end
result.solver.success=true;result.status=struct('success',true,'code','solved','message','The 500-node canonical boundary-value equations converged for the full obstacle objective.');
end
