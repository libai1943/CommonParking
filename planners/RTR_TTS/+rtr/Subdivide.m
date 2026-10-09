function [u,info]=Subdivide(path,polygons,body,o)
% Recursive geometric-path approximation; exact eeS ensures the paper's limit.
radius=hypot(max(body(1:2)),body(3));delta=diff(path);step=vecnorm(delta(:,1:2),2,2)+radius*abs(delta(:,3));path=path([true;step>1e-11],:);
delta=diff(path);step=vecnorm(delta(:,1:2),2,2)+radius*abs(delta(:,3));s=[0;cumsum(step)];clock=tic;calls=0;deepest=0;accepted={};reverseCount=0;success=true;
try,u=split(0,s(end),0);catch problem
 if strcmp(problem.identifier,'RTR_TTS:Budget'),u=zeros(0,3);success=false;else,rethrow(problem);end
end
info=struct('success',success,'calls',calls,'maximum_depth',deepest,'accepted_types',{accepted},'reversed_connections',reverseCount,'time_s',toc(clock));
 function controls=split(left,right,depth)
  calls=calls+1;deepest=max(deepest,depth);if calls>o.subdivisionCalls||toc(clock)>o.subdivisionSeconds,error('RTR_TTS:Budget','Finite subdivision budget exhausted.');end
  a=interp1(s,path,left);b=interp1(s,path,right);controls=[];
  for reversed=0:1
   if reversed,from=b;to=a;else,from=a;to=b;end
   [paths,kinds]=rtr.Connector(from,to,o);
   for j=1:numel(paths)
    p=paths{j};if ~ccp.Edge([from,0],p,polygons,body,o),continue;end
    if reversed
     last=p(:,2)+p(:,3).*abs(p(:,1));p=flipud([-p(:,1),last,-p(:,3)]);reverseCount=reverseCount+1;
    end
    controls=p;accepted{end+1}=kinds{j};return;
   end
  end
  if depth>=o.subdivisionDepth,error('RTR_TTS:Budget','Finite subdivision depth exhausted.');end
  mid=(left+right)/2;controls=[split(left,mid,depth+1);split(mid,right,depth+1)];
 end
end
