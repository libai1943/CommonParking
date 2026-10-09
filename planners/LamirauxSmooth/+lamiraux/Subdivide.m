function [pieces,info]=Subdivide(path,c,o)
clock=tic;radius=hypot(max(c.vehicle.lr,c.vehicle.lw+c.vehicle.lf),c.vehicle.lb/2);
steps=vecnorm(diff(path(:,1:2)),2,2)+radius*abs(diff(path(:,3)));path=path([true;steps>1e-12],:);
steps=vecnorm(diff(path(:,1:2)),2,2)+radius*abs(diff(path(:,3)));parameter=[0;cumsum(steps)];calls=0;deepest=0;accepted=0;success=true;
try
 % Section IV-A starts with a midpoint and connects each half separately.
 middle=parameter(end)/2;pieces=[split(0,middle,1),split(middle,parameter(end),1)];
catch problem
 if strcmp(problem.identifier,'LamirauxSmooth:SubdivisionBudget'),pieces=struct([]);success=false;else,rethrow(problem);end
end
info=struct('success',success,'connection_calls',calls,'maximum_depth',deepest,'accepted_connections',accepted,'time_s',toc(clock));
 function output=split(left,right,depth)
  calls=calls+1;deepest=max(deepest,depth);
  if calls>o.subdivisionCalls||toc(clock)>o.subdivisionSeconds,error('LamirauxSmooth:SubdivisionBudget','Subdivision resource limit.');end
  a=[interp1(parameter,path,left,'linear'),0];b=[interp1(parameter,path,right,'linear'),0];
  [output,ok]=lamiraux.Steer(a,b,c,o);if ok,accepted=accepted+1;return;end
  if depth>=o.subdivisionDepth,error('LamirauxSmooth:SubdivisionBudget','Subdivision depth limit.');end
  mid=(left+right)/2;output=[split(left,mid,depth+1),split(mid,right,depth+1)];
 end
end
