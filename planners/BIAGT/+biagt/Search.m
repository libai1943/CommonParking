function [arcs,info]=Search(c,o)
start=[c.task.x0 c.task.y0 c.task.theta0];goal=[c.task.xf c.task.yf c.task.thetaf];v=c.vehicle;
data=parking.PolygonData(c);polygons=data.vertices;body=[v.lw+v.lf v.lr v.lb/2];xy=[start(1:2);goal(1:2);vertcat(polygons{:})];bounds=[min(xy)-o.padding;max(xy)+o.padding];
T={newTree(start,goal,o,c),newTree(goal,start,o,c)};clock=tic;arcs=[];iteration=0;last=2;close=false;attachmentAttempts=0;winner=0;endNode=0;
while iteration<o.iterations&&toc(clock)<o.seconds&&sum(cellfun(@(a)a.n,T))<o.maxNodes
 iteration=iteration+1;[f1,b1]=min(T{1}.key(1:T{1}.n));[f2,b2]=min(T{2}.key(1:T{2}.n));
 if ~isfinite(f1)&&~isfinite(f2),break;end
 if ~isfinite(f1),side=2;elseif ~isfinite(f2),side=1;
 elseif ~close,side=3-last;
 else
  % The paper says lower estimated cost-to-go, not lower complete F-value.
  hs=[T{1}.h(b1),T{2}.h(b2)];if hs(1)==hs(2),side=3-last;else,[~,side]=min(hs);end
 end
 last=side;A=T{side};B=T{3-side};if side==1,best=b1;else,best=b2;end
 q=A.q(best,:);
 if ~A.endTried(best)&&biagt.Distance(q,A.target,o.metric)<=o.epsilon
  A.endTried(best)=true;attachmentAttempts=attachmentAttempts+1;[~,edges]=biagt_rs_mex(q,A.target,v.kappa_max);edge=edges{1};
  if biagt.Free(q,edge,c,o,body,polygons)
   arcs=[extract(A,best);edge];if side==2,arcs=flipud(arcs);arcs(:,1)=-arcs(:,1);end
   winner=side;endNode=best;T{side}=A;break;
  end
 end
 if ~any(A.tried(best,:))&&A.parent(best)>0,A.priority(best,:)=A.priority(A.parent(best),:);end
 available=find(~A.tried(best,:));[~,j]=min(A.priority(best,available));mode=available(j);A.tried(best,mode)=true;direction=3-2*mode;
 childF=[];
 for curvature=o.normalizedSteering*v.kappa_max
  edge=[direction*o.length curvature];next=biagt.Advance(q,edge(1),edge(2));
  if any(next(1:2)<bounds(1,:))||any(next(1:2)>bounds(2,:))||any(biagt.Distance(next,A.q(1:A.n,:),o.metric)<o.delta),continue;end
  if ~biagt.Free(q,edge,c,o,body,polygons),continue;end
  if ~close&&any(biagt.Distance(next,B.q(1:B.n,:),o.metric)<=o.mu),close=true;end
  [h,source]=biagt.Heuristic(next,A.target,B,c,o);g=A.g(best)+o.length;f=g+h;
  childF(end+1)=f;A.priority(best,mode)=double(min(childF)>=A.g(best)+A.h(best)); %#ok<AGROW>
  A.n=A.n+1;k=A.n;A.q(k,:)=next;A.parent(k)=best;A.arc(k,:)=edge;A.g(k)=g;A.h(k)=h;A.key(k)=f;A.source(k)=source;A.otherCount(k)=B.n;
  A.priority(k,:)=A.priority(best,:);A.tried(k,:)=false;A.endTried(k)=false;
 end
 if all(A.tried(best,:)),A.key(best)=inf;end
 T{side}=A;
 if mod(iteration,1000)==0,fprintf('BIAGT %d expansions, %d+%d nodes, %.1f seconds\n',iteration,T{1}.n,T{2}.n,toc(clock));end
end
for k=1:2,T{k}=trim(T{k});end
info=struct('iterations',iteration,'seconds',toc(clock),'nodes',[T{1}.n,T{2}.n],'trees',{T},'close',close,'attachment_attempts',attachmentAttempts,'winner',winner,'end_node',endNode,'bounds',bounds);
end
function A=newTree(root,target,o,c)
n=2048;A=struct('n',1,'root',root,'target',target,'q',zeros(n,3),'g',zeros(n,1),'h',zeros(n,1),'key',inf(n,1),'parent',zeros(n,1),'arc',zeros(n,2),'priority',ones(n,2),'tried',false(n,2),'endTried',false(n,1),'source',zeros(n,1),'otherCount',zeros(n,1));
A.q(1,:)=root;A.h(1)=o.rho*biagt_rs_mex(root,target,c.vehicle.kappa_max);A.key(1)=A.h(1);
end
function p=extract(A,k)
p=zeros(0,2);while A.parent(k)>0,p=[A.arc(k,:);p];k=A.parent(k);end %#ok<AGROW>
end
function A=trim(A)
for field={'q','g','h','key','parent','arc','priority','tried','endTried','source','otherCount'},f=field{1};A.(f)=A.(f)(1:A.n,:);end
end
