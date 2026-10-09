function [z,info]=Optimize(z0,lower,upper,c,gears,o)
timer=tic;lastZ=[];lastCost=[];lastConstraint=[];lastValues=[];history=zeros(0,3);
settings=optimoptions('fmincon','Algorithm','sqp','Display','none','MaxIterations',o.maxIterations, ...
 'MaxFunctionEvaluations',o.maxEvaluations,'ConstraintTolerance',1e-6,'OptimalityTolerance',1e-6, ...
 'StepTolerance',1e-9,'FiniteDifferenceType','forward','OutputFcn',@stop);
[z,cost,flag,output]=fmincon(@objective,z0,[],[],[],[],lower,upper,@constraints,settings);
[~,g,values]=eta3.Measure(z,c,gears,o);residual=max([0;g;lower-z;z-upper]);
info=struct('success',flag>0&&residual<=1e-6,'exitflag',flag,'message',output.message,'iterations',output.iterations, ...
 'function_evaluations',output.funcCount,'maximum_violation',residual,'objective',cost,'objectives',values,'history',history);
 function update(candidate)
  if ~isequal(candidate,lastZ),[lastCost,lastConstraint,lastValues]=eta3.Measure(candidate,c,gears,o);lastZ=candidate;end
 end
 function f=objective(candidate),update(candidate);f=lastCost;end
 function [g,eq]=constraints(candidate),update(candidate);g=lastConstraint;eq=[];end
 function halted=stop(~,values,state)
  halted=toc(timer)>o.maxSeconds;
  if strcmp(state,'iter'),history(end+1,:)=[values.iteration,values.fval,values.constrviolation];end
 end
end
