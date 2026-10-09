function yes=SegmentFree(a,b,env)
% Exact line-segment / convex-polygon intersection for the holonomic graph.
yes=true;delta=b-a;
for j=1:numel(env.polygons)
 p=env.polygons{j};area=sum(p(:,1).*p([2:end 1],2)-p(:,2).*p([2:end 1],1));
 enter=0;leave=1;
 for k=1:size(p,1)
  edge=p(mod(k,size(p,1))+1,:)-p(k,:);normal=sign(area)*[edge(2),-edge(1)];
  offset=dot(normal,a-p(k,:));slope=dot(normal,delta);
  if abs(slope)<1e-12
   if offset>0,enter=Inf;break;end
  elseif slope>0,leave=min(leave,-offset/slope);else,enter=max(enter,-offset/slope);end
 end
 if enter<=leave&&leave>=0&&enter<=1,yes=false;return;end
end
end
