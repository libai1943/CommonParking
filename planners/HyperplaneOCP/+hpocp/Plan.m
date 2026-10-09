function result=Plan(c)
o=hpocp.Config();result=cp.EmptyResult('HyperplaneOCP',c.id,'trajectory');previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
[z,u,h,planes]=hpocp.Initialize(c,o);result.diagnostics=struct('options',o,'initial_states',z,'initial_controls',u,'initial_h',h,'initial_planes',planes);
[q,solver,native]=hpocp.Solve(c,z,u,h,planes,o);result.solver=solver;result.diagnostics.native=native;
if ~solver.success,result.status.code='optimization_failed';result.status.message=solver.message;return;end
result.trajectory=q;result.status=struct('success',true,'code','solved','message','Primal separating-hyperplane RK4 optimal control converged and passed its native equation checks.');
end
