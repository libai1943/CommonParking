function [path,info]=Global(start,goal,polygons,body,o)
% Paper Algorithm 2 and Section 5.3: bidirectional RT trees with TCI interiors.
clock=tic;xy=[start(1:2);goal(1:2);vertcat(polygons{:})];lower=min(xy)-o.padding;upper=max(xy)+o.padding;
trees={init(start),init(goal)};path=[];iterations=0;
if ~rtr.GeometricFree(start,start,polygons,body,o)||~rtr.GeometricFree(goal,goal,polygons,body,o),info=statistics();return;end
[path,joined]=connect(1,find(trees{1}.type==1));
while ~joined&&iterations<o.iterations&&toc(clock)<o.globalSeconds
 iterations=iterations+1;t=1+mod(iterations-1,2);p=lower+rand(1,2).*(upper-lower);
 [tree,near]=nearest(trees{t},p);trees{t}=tree;q=tree.q(near,:);desired=atan2(p(2)-q(2),p(1)-q(1));delta=atan2(sin(desired-q(3)),cos(desired-q(3)));
 [new,blocked]=extend(t,near,delta);[path,joined]=connect(t,new);
 if blocked&&~joined
  if delta>=0,opposite=delta-2*pi;else,opposite=delta+2*pi;end
  [new,~]=extend(t,near,opposite);[path,joined]=connect(t,new);
 end
end
if ~isempty(path)
 for j=2:size(path,1),path(j,3)=path(j-1,3)+atan2(sin(path(j,3)-path(j-1,3)),cos(path(j,3)-path(j-1,3)));end
end
info=statistics();
 function out=statistics()
  out=struct('success',~isempty(path),'iterations',iterations,'vertices',[size(trees{1}.q,1),size(trees{2}.q,1)],'time_s',toc(clock),'bounds',[lower;upper]);
 end
 function tree=init(q)
  tree=struct('q',q,'parent',0,'type',0);for d=[1,-1],b=translate(q,d);if norm(b(1:2)-q(1:2))>1e-8,tree=add(tree,b,1,1);end,end
 end
 function tree=add(tree,q,parent,type)
  tree.q(end+1,:)=q;tree.parent(end+1,1)=parent;tree.type(end+1,1)=type;
 end
 function [tree,index]=nearest(tree,p)
  metric=sum((tree.q(:,1:2)-p).^2,2);[best,index]=min(metric);child=find(tree.type==1);parents=tree.parent(child);a=tree.q(parents,1:2);b=tree.q(child,1:2);v=b-a;
  fraction=max(0,min(1,sum((p-a).*v,2)./sum(v.^2,2)));point=a+fraction.*v;distance=sum((point-p).^2,2);[value,k]=min(distance);
  if isempty(value)||value>=best,return;end
  if fraction(k)<1e-9,index=parents(k);elseif fraction(k)>1-1e-9,index=child(k);else
   q=[point(k,:),tree.q(child(k),3)];tree=add(tree,q,parents(k),1);index=size(tree.q,1);tree.parent(child(k))=index;
  end
 end
 function [added,blocked]=extend(t,parent,delta)
  tree=trees{t};q=tree.q(parent,:);[seq,blocked]=rotate(q,delta);for j=2:size(seq,1),tree=add(tree,seq(j,:),parent,2);parent=size(tree.q,1);end
  added=[];q=tree.q(parent,:);
  for d=[1,-1]
   z=translate(q,d);if norm(z(1:2)-q(1:2))>1e-8,tree=add(tree,z,parent,1);added(end+1)=size(tree.q,1);end %#ok<AGROW>
  end
  trees{t}=tree;
 end
 function z=translate(q,d)
  direction=d*[cos(q(3)),sin(q(3))];distance=inf;
  for ax=1:2
   if direction(ax)>1e-12,distance=min(distance,(upper(ax)-q(ax))/direction(ax));elseif direction(ax)<-1e-12,distance=min(distance,(lower(ax)-q(ax))/direction(ax));end
  end
  distance=max(0,distance);last=q;z=q;
  for h=linspace(0,distance,max(1,ceil(distance/o.translationStep))+1)
   target=q+[h*direction,0];
   if ~rtr.GeometricFree(last,target,polygons,body,o),z=boundary(last,target);return;end
   last=target;z=target;
  end
 end
 function [seq,blocked]=rotate(q,delta)
  seq=q;blocked=false;n=max(1,ceil(abs(delta)/o.rotationStep));
  for j=1:n
   target=q+[0,0,delta*j/n];
   if ~rtr.GeometricFree(seq(end,:),target,polygons,body,o)
    target=boundary(seq(end,:),target);if abs(target(3)-seq(end,3))>1e-9,seq(end+1,:)=target;end;blocked=true;return;
   end
   if abs(target(3)-seq(end,3))>1e-9,seq(end+1,:)=target;end
  end
 end
 function q=boundary(a,b)
  lo=0;hi=1;motion=norm(b(1:2)-a(1:2))+hypot(max(body(1:2)),body(3))*abs(b(3)-a(3));
  while (hi-lo)*motion>o.boundaryTolerance
   mid=(lo+hi)/2;if rtr.GeometricFree(a,a+mid*(b-a),polygons,body,o),lo=mid;else,hi=mid;end
  end
  q=a+lo*(b-a);
 end
 function [path,ok]=connect(t,new)
  path=[];ok=false;first=trees{t};second=trees{3-t};other=find(second.type==1);
  depth=zeros(size(other));for j=1:numel(other),node=other(j);while node>1,depth(j)=depth(j)+1;node=second.parent(node);end,end;[~,ix]=sort(depth);other=other(ix);
  for aa=new(:)'
   a=first.q(first.parent(aa),:);b=first.q(aa,:);
   for bb=other(:)'
    c=second.q(second.parent(bb),:);d=second.q(bb,:);[point,hit]=intersection(a(1:2),b(1:2),c(1:2),d(1:2));if ~hit,continue;end
    qa=[point,a(3)];qb=[point,c(3)];delta=atan2(sin(qb(3)-qa(3)),cos(qb(3)-qa(3)));[seq,blocked]=rotate(qa,delta);
    if blocked,if delta>=0,delta=delta-2*pi;else,delta=delta+2*pi;end;[seq,blocked]=rotate(qa,delta);end
    if blocked,continue;end
    p1=[trace(first,first.parent(aa));qa];p2=[trace(second,second.parent(bb));qb];path=[p1;seq(2:end,:);flipud(p2)];if t==2,path=flipud(path);end;ok=true;return;
   end
  end
 end
end
function path=trace(tree,index)
route=index;while tree.parent(index)>0,index=tree.parent(index);route(end+1)=index;end;path=tree.q(fliplr(route),:);
end
function [point,hit]=intersection(a,b,c,d)
v=b-a;w=d-c;cross=@(u,v)u(1)*v(2)-u(2)*v(1);den=cross(v,w);point=[NaN,NaN];hit=false;
if abs(den)>1e-12
 t=cross(c-a,w)/den;s=cross(c-a,v)/den;if t>=-1e-10&&t<=1+1e-10&&s>=-1e-10&&s<=1+1e-10,point=a+max(0,min(1,t))*v;hit=true;end
elseif abs(cross(c-a,v))<1e-10&&dot(v,v)>1e-14
 t=sort([dot(c-a,v),dot(d-a,v)]/dot(v,v));lo=max(0,t(1));hi=min(1,t(2));if lo<=hi+1e-10,point=a+lo*v;hit=true;end
end
end
