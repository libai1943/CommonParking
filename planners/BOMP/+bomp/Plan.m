function result = Plan(c)
options=bomp.Config();result=cp.EmptyResult('BOMP',c.id,'trajectory');
[trajectory,solver,native]=bomp.Solve(c,options);
result.solver=solver;result.diagnostics=struct('native',native,'options',options);
if ~isempty(trajectory),result.trajectory=trajectory;end
if solver.success,result.status=struct('success',true,'code','solved','message',solver.message);
else,result.status.code='optimization_failed';result.status.message=solver.message;end
end
