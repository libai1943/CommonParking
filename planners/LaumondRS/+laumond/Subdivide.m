function [primitives,info]=Subdivide(path,c,options,connection)
clock=tic;radius=hypot(max(c.vehicle.lr,c.vehicle.lw+c.vehicle.lf),c.vehicle.lb/2);
step=vecnorm(diff(path(:,1:2)),2,2)+radius*abs(diff(path(:,3)));path=path([true;step>1e-12],:);
step=vecnorm(diff(path(:,1:2)),2,2)+radius*abs(diff(path(:,3)));parameter=[0;cumsum(step)];
calls=0;deepest=0;pieces=0;success=true;
try
 primitives=split(0,parameter(end),0);
catch problem
 if strcmp(problem.identifier,'LaumondRS:SubdivisionBudget'),primitives=zeros(0,2);success=false;else,rethrow(problem);end
end
info=struct('success',success,'connection_calls',calls,'maximum_depth',deepest,'accepted_pieces',pieces,'time_s',toc(clock));
 function output=split(left,right,depth)
  calls=calls+1;deepest=max(deepest,depth);
  if calls>options.subdivisionCalls||toc(clock)>options.subdivisionSeconds,error('LaumondRS:SubdivisionBudget','Subdivision resource limit.');end
  a=interp1(parameter,path,left,'linear');b=interp1(parameter,path,right,'linear');
  output=laumond.Shortest(connection,a,b,c.vehicle.kappa_max);
  if laumond.ArcsFree(a,output,c),pieces=pieces+1;return;end
  if depth>=options.subdivisionDepth,error('LaumondRS:SubdivisionBudget','Subdivision depth limit.');end
  middle=(left+right)/2;first=split(left,middle,depth+1);second=split(middle,right,depth+1);output=[first;second];
 end
end
