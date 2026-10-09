function [phases,info]=Subdivide(path,v,polygons,body,o)
radius=hypot(max(body(1:2)),body(3));delta=diff(path);step=vecnorm(delta(:,1:2),2,2)+radius*abs(delta(:,3));path=path([true;step>1e-11],:);delta=diff(path);s=[0;cumsum(vecnorm(delta(:,1:2),2,2)+radius*abs(delta(:,3)))];
clock=tic;calls=0;deepest=0;accepted=0;success=true;
try,phases=split(0,s(end),0);catch problem
 if strcmp(problem.identifier,'Sinusoid_RTR:Budget'),phases=[];success=false;else,rethrow(problem);end
end
info=struct('success',success,'connection_calls',calls,'maximum_depth',deepest,'accepted_connections',accepted,'time_s',toc(clock));
 function out=split(left,right,depth)
  calls=calls+1;deepest=max(deepest,depth);if calls>o.subdivisionCalls||toc(clock)>o.subdivisionSeconds,error('Sinusoid_RTR:Budget','Finite subdivision budget exhausted.');end
  a=interp1(s,path,left);b=interp1(s,path,right);paths=sinsteer.Connector(a,b,v,o);
  for k=1:numel(paths),candidate=paths{k};if sinsteer.Edge(candidate,polygons,body,o),out=candidate;accepted=accepted+1;return;end,end
  if depth>=o.subdivisionDepth,error('Sinusoid_RTR:Budget','Finite subdivision depth exhausted.');end
  mid=(left+right)/2;out=[split(left,mid,depth+1),split(mid,right,depth+1)];
 end
end
