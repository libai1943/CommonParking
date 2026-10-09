function result=Plan(c)
o=dftpav.Config();result=cp.EmptyResult('DFTPAV_Path',c.id,'path');old=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(old)); %#ok<NASGU>
raw=parking.SearchHybridAStar(c,o.search);result.diagnostics=struct('options',o,'search',raw);
if ~raw.success,result.status.code='initialization_failed';result.status.message='Hybrid A* did not connect the task.';return;end
path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],raw.primitives,c.vehicle,.05);[z0,problem]=dftpav.Prepare(path,c,o);
clock=tic;history=[];options=optimoptions('fminunc','Algorithm','quasi-newton','SpecifyObjectiveGradient',true,'Display','off', ...
 'MaxIterations',o.maximumIterations,'MaxFunctionEvaluations',20000,'OptimalityTolerance',1e-6,'FunctionTolerance',1e-8,'StepTolerance',1e-12,'OutputFcn',@progress);
[z,f,flag,output]=fminunc(@(x)dftpav.Objective(x,problem),z0,options);[~,gradient,detail]=dftpav.Objective(z,problem);
success=flag>0&&isfinite(f)&&f<o.maximumObjective;
result.solver=struct('name','MATLAB fminunc quasi-Newton BFGS','success',success,'exitflag',flag,'iterations',output.iterations,'objective',f,'gradient_infinity_norm',norm(gradient,Inf),'message',output.message);
result.diagnostics.problem=problem;result.diagnostics.initial_variables=z0;result.diagnostics.variables=z;result.diagnostics.native=detail;result.diagnostics.history=history;
if success
 try
  [result.path,result.diagnostics.native_time_law]=dftpav.Export(detail,c.vehicle);
  result.status=struct('success',true,'code','solved','message','The paper static MINCO/gear-shift optimization converged; its optimized geometry is submitted as a path. Soft constraints are not a feasibility certificate.');
 catch failure
  result.solver.success=false;result.status.code='geometric_export_failed';result.status.message=failure.message;
 end
else,result.status.code='optimization_failed';result.status.message=output.message;end
 function stop=progress(~,values,state)
  stop=toc(clock)>o.maximumSeconds;if strcmp(state,'iter'),history(end+1,:)=[values.iteration values.fval values.firstorderopt];end
 end
end
