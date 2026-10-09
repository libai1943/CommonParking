function result=Search(c,o)
% Algorithm 1: two RS trees, a third smoothing tree, and node feedback.
result=cp.EmptyResult('SmoothBiRRT',c.id,'path');v=c.vehicle;polygons=parking.PolygonData(c);
radius=v.lb/2+o.sideMirrorMargin+o.buffer;count=max(1,ceil(v.length/(2*sqrt(radius^2-(v.lb/2)^2))));
centres=-v.lr+((1:count)-.5)*v.length/count;
start=[c.task.x0 c.task.y0 c.task.theta0];goal=[c.task.xf c.task.yf c.task.thetaf];
points=[start(1:2);goal(1:2);vertcat(polygons.vertices{:})];bounds=[min(points)-o.padding;max(points)+o.padding];initialBounds=bounds;
result.diagnostics=struct('options',o,'disc_count',count,'disc_centres',centres,'disc_radius',radius,'initial_bounds',initialBounds);
if ~sfrrt.DiscsFree([start;goal],polygons.vertices,centres,radius)
 result.status.code='endpoint_disc_collision';result.status.message='A required endpoint intersects the paper-style enclosing-disc model.';return;
end
old=rng;restore=onCleanup(@()rng(old));rng(o.seedBase+c.id,'twister'); %#ok<NASGU>
connection=reedsSheppConnection('MinTurningRadius',1/v.kappa_max,'ForwardCost',1,'ReverseCost',1);
trees={newTree(start),newTree(goal)};best=inf;bestArcs=zeros(0,2);bestHistory=[];iterations=0;rewires=0;connections=0;feedbackNodes=0;smoothingRuns=0;rsCalls=0;firstTime=nan;clock=tic;
while toc(clock)<o.seconds&&iterations<o.maximumIterations
 iterations=iterations+1;[trees{1},first]=insert(trees{1},sample(goal),1,0,[]);
 [trees{2},~]=insert(trees{2},sample(start),2,0,[]);
 if first==0,continue;end
 [trees{2},second]=insert(trees{2},trees{1}.q(first,:),2,0,[]);
 if second==0,continue;end
 connections=connections+1;arcs=[branch(trees{1},first);sfrrt.Reverse(branch(trees{2},second))];
 [arcs,smoothNodes]=smooth(arcs);smoothingRuns=smoothingRuns+1;
 % Feedback is performed for every connected, smoothed solution, not only
 % for the incumbent. Both trees receive the physical reverse consistently.
 feedback(1,start,arcs,smoothNodes);feedback(2,goal,sfrrt.Reverse(arcs),flipud(sum(abs(arcs(:,1)))-smoothNodes));
 value=sfrrt.Cost(arcs,o.weights);
 if value<best-1e-10
  best=value;bestArcs=arcs;if isnan(firstTime),firstTime=toc(clock);end
  q=cp.ArcPose(start,arcs,linspace(0,sum(abs(arcs(:,1))),max(2,ceil(sum(abs(arcs(:,1)))/.1)+1))');
  bounds=[max(initialBounds(1,:),min(q(:,1:2))-o.roiPadding);min(initialBounds(2,:),max(q(:,1:2))+o.roiPadding)];
  bestHistory(end+1,:)=[toc(clock),best]; %#ok<AGROW>
 end
end
result.solver=struct('success',isfinite(best),'iterations',iterations,'nodes',[size(trees{1}.q,1),size(trees{2}.q,1)],'rewires',rewires,'connections',connections,'smoothing_runs',smoothingRuns,'feedback_nodes',feedbackNodes,'rs_calls',rsCalls,'cost',best,'search_time_s',toc(clock),'time_to_first_solution_s',firstTime,'seed',o.seedBase+c.id);
result.diagnostics.primitives=bestArcs;result.diagnostics.best_history=bestHistory;result.diagnostics.final_bounds=bounds;
if isempty(bestArcs),result.status.code='search_budget_exhausted';result.status.message='No connected disc-checked path was obtained within the disclosed budget.';return;end
result.path=cp.PathFromArcs(start,bestArcs,v,o.outputSpacing);
endpoint=[result.path.x(end)-goal(1),result.path.y(end)-goal(2),atan2(sin(result.path.theta(end)-goal(3)),cos(result.path.theta(end)-goal(3)))];assert(max(abs(endpoint))<1e-6);
result.status=struct('success',true,'code','path_found','message','Revised bidirectional RS RRT*, third-tree smoothing, feedback and ROI updates completed. Collision checks use sampled enclosing discs.');

 function T=newTree(q)
  T=struct('q',q,'parent',0,'arcs',{{zeros(0,2)}},'cost',0);
 end
 function q=sample(other)
  if rand<o.goalBias,q=other;else,q=[bounds(1,:)+rand(1,2).*(bounds(2,:)-bounds(1,:)),2*pi*rand-pi];end
 end
 function pp=branch(T,node)
  pp=zeros(0,2);while node>1,pp=[T.arcs{node};pp];node=T.parent(node);end %#ok<AGROW>
 end
 function cost=physicalCost(arcs,side)
  if side==2,arcs=sfrrt.Reverse(arcs);end;cost=sfrrt.Cost(arcs,o.weights);
 end
 function [edge,value]=steer(T,parent,q,side)
  rsCalls=rsCalls+1;[paths,lengths]=connect(connection,T.q(parent,:),q,'PathSegments','all');prefix=branch(T,parent);edge=zeros(0,2);value=inf;
  for r=1:numel(paths)
   if ~isfinite(lengths(r))||lengths(r)>o.rewireRadius+1e-10||lengths(r)<1e-9,continue;end
   p=paths{r};k=zeros(numel(p.MotionLengths),1);k(strcmp(p.MotionTypes,'L'))=v.kappa_max;k(strcmp(p.MotionTypes,'R'))=-v.kappa_max;
   candidate=[p.MotionLengths(:).*p.MotionDirections(:),k];candidate(abs(candidate(:,1))<1e-10,:)=[];
   candidateCost=physicalCost([prefix;candidate],side);
   if candidateCost<value,value=candidateCost;edge=candidate;end
  end
 end
 function free=edgeFree(q,arcs)
  length=sum(abs(arcs(:,1)));if isempty(arcs),free=false;return;end
  mile=unique([linspace(0,length,max(1,ceil(length/o.collisionSpacing))+1)';cumsum(abs(arcs(:,1)))]);
  poses=cp.ArcPose(q,arcs,mile);free=sfrrt.DiscsFree(poses,polygons.vertices,centres,radius);
 end
 function [T,current]=insert(T,q,side,fallbackParent,fallbackArc)
  current=0;if ~sfrrt.DiscsFree(q,polygons.vertices,centres,radius),return;end
  distance=hypot(T.q(:,1)-q(1),T.q(:,2)-q(2));same=find(distance<1e-8&abs(atan2(sin(T.q(:,3)-q(3)),cos(T.q(:,3)-q(3))))<1e-8);
  if ~isempty(same),[~,pick]=min(T.cost(same));current=same(pick);return;end
  parent=fallbackParent;chosen=fallbackArc;value=inf;
  if parent>0,value=physicalCost([branch(T,parent);chosen],side);end
  near=find(distance<=o.rewireRadius)';
  for node=near
   if toc(clock)>=o.seconds,break;end
   [edge,cost]=steer(T,node,q,side);
   if cost<value&&edgeFree(T.q(node,:),edge),parent=node;chosen=edge;value=cost;end
  end
  if parent==0,return;end
  current=size(T.q,1)+1;T.q(current,:)=q;T.parent(current,1)=parent;T.arcs{current,1}=chosen;T.cost(current,1)=value;
  ancestors=current;at=parent;while at>0,ancestors(end+1)=at;at=T.parent(at);end %#ok<AGROW>
  for node=near
   if toc(clock)>=o.seconds,break;end
   if ismember(node,ancestors),continue;end
   [edge,cost]=steer(T,current,T.q(node,:),side);
   if cost+1e-10<T.cost(node)&&edgeFree(q,edge)
    T.parent(node)=current;T.arcs{node}=edge;rewires=rewires+1;todo=node;
    while ~isempty(todo),at=todo(1);todo(1)=[];T.cost(at)=physicalCost(branch(T,at),side);todo=[todo,find(T.parent==at)'];end %#ok<AGROW>
   end
  end
 end
 function [pp,miles]=smooth(original)
  length=sum(abs(original(:,1)));miles=unique([0;linspace(0,length,max(1,ceil(length/o.interpolationSpacing))+1)';cumsum(abs(original(:,1)))]);
  poses=cp.ArcPose(start,original,miles);T=newTree(start);last=1;at=1;
  for k=2:numel(miles)
   if toc(clock)>=o.seconds,break;end
   segment=sfrrt.Subpath(original,miles(k-1),miles(k));[T,last]=insert(T,poses(k,:),1,last,segment);
   if last==0,pp=original;return;end;at=k;
  end
  pp=[branch(T,last);sfrrt.Subpath(original,miles(at),length)];
  if sfrrt.Cost(pp,o.weights)>sfrrt.Cost(original,o.weights)+1e-9,pp=original;end
  % These are samples of the resulting path, including every exact arc end.
  length=sum(abs(pp(:,1)));miles=unique([0;linspace(0,length,max(1,ceil(length/o.interpolationSpacing))+1)';cumsum(abs(pp(:,1)))]);
 end
 function feedback(side,root,pp,miles)
  T=trees{side};poses=cp.ArcPose(root,pp,miles);parent=1;
  for j=2:numel(miles)
   node=size(T.q,1)+1;T.q(node,:)=poses(j,:);T.parent(node,1)=parent;T.arcs{node,1}=sfrrt.Subpath(pp,miles(j-1),miles(j));T.cost(node,1)=physicalCost(branch(T,node),side);parent=node;
  end
  feedbackNodes=feedbackNodes+numel(miles)-1;trees{side}=T;
 end
end
