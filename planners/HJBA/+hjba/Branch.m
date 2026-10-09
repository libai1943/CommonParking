function out=Branch(c,connected,bounds,o)
% Algorithms 1-2: two independent searches toward ONE connected state.
previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
t=c.task;roots={[t.x0 t.y0 t.theta0],[t.xf t.yf t.thetaf]};res=[o.forwardResolution,o.backwardResolution];
connection=reedsSheppConnection('MinTurningRadius',1/c.vehicle.kappa_max,'ForwardCost',1,'ReverseCost',1);
trees={newTree(roots{1}),newTree(roots{2})};found=false(1,2);arcs=cell(1,2);joins=cell(1,2);traces=cell(1,2);expanded=zeros(1,2);reverseFallbacks=0;clock=tic;
while ~all(found)&&sum(expanded)<o.maximumExpanded&&toc(clock)<o.branchSeconds
 if any(cellfun(@(a)~any(isfinite(a.f)),trees)&~found),break;end
 for side=1:2
  if found(side),continue;end
  T=trees{side};[~,at]=min(T.f);T.f(at)=inf;cellId=key(T.q(at,:),side);T.closed(cellId)=true;expanded(side)=expanded(side)+1;q=T.q(at,:);
  edge=laumond.Shortest(connection,q,connected,c.vehicle.kappa_max);
  if laumond.ArcsFree(q,edge,c)
   [prefix,trace]=backtrack(T,at);arcs{side}=[prefix;edge];joins{side}=q;traces{side}=trace;found(side)=true;trees{side}=T;continue;
  end
  steering=linspace(-c.vehicle.phimax,c.vehicle.phimax,o.steeringSamples);curv=tan(steering)/c.vehicle.lw;
  forwardValid=false;
  for k=1:numel(curv),forwardValid=extend(o.stepFactor*res(side),curv(k))||forwardValid;end
  if side==2
   for k=1:numel(curv),extend(-o.stepFactor*res(side),curv(k));end
  elseif ~forwardValid
   reverseFallbacks=reverseFallbacks+1;extend(-o.stepFactor*res(side),curv(1));extend(-o.stepFactor*res(side),curv(end));
  end
  trees{side}=T;
 end
end
out=struct('success',all(found),'expanded',expanded,'reverse_fallbacks',reverseFallbacks,'search_time_s',toc(clock),'connected',connected,'cost',inf,'primitives',zeros(0,2),'joins',{joins},'traces',{traces});
if all(found),out.primitives=[arcs{1};[-flipud(arcs{2}(:,1)),flipud(arcs{2}(:,2))]];out.cost=sum(abs(out.primitives(:,1)));end

 function T=newTree(q)
  T=struct('q',q,'g',0,'f',heuristic(q),'parent',0,'primitive',[0 0], ...
   'open',containers.Map('KeyType','uint64','ValueType','double'),'closed',containers.Map('KeyType','uint64','ValueType','logical'));
  s=1;if isequal(q,roots{2}),s=2;end;T.open(key(q,s))=1;
 end
 function id=key(q,s)
  xy=floor((q(1:2)-bounds(1,:))/res(s));sz=ceil((bounds(2,:)-bounds(1,:))/res(s))+1;
  a=mod(floor(mod(q(3),2*pi)/o.headingResolution),round(2*pi/o.headingResolution));id=uint64(1+xy(1)+sz(1)*(xy(2)+sz(2)*a));
 end
 function h=heuristic(q)
  d=abs(q(1:2)-connected(1:2));h=max(d)+(sqrt(2)-1)*min(d);
 end
 function valid=extend(length,curvature)
  q2=parking.IntegratePrimitive(q,length,curvature);valid=false;
  if any(q2(1:2)<bounds(1,:))||any(q2(1:2)>bounds(2,:))||~laumond.ArcsFree(q,[length curvature],c),return;end
  valid=true;id=key(q2,side);if isKey(T.closed,id),return;end
  cost=T.g(at)+abs(length);
  if isKey(T.open,id)
   old=T.open(id);if T.g(old)<=cost,return;end;T.f(old)=inf;
  end
  index=size(T.q,1)+1;T.q(index,:)=q2;T.g(index,1)=cost;T.f(index,1)=cost+heuristic(q2);T.parent(index,1)=at;T.primitive(index,:)=[length curvature];T.open(id)=index;
 end
 function [p,trace]=backtrack(T,index)
  chain=index;while T.parent(chain(1))>0,chain=[T.parent(chain(1)),chain];end %#ok<AGROW>
  p=T.primitive(chain(2:end),:);trace=struct('q',T.q(chain,:),'g',T.g(chain),'primitives',p);
 end
end
