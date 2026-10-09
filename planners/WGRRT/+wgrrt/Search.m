function [graph,report]=Search(W,c,o,connection)
% Algorithm 5: retain the accumulated start graph, reset only the next goal tree.
clock=tic;graph=struct('q',W(1,:),'from',zeros(0,1),'to',zeros(0,1),'cost',zeros(0,1), ...
 'primitives',{{}},'start',1,'goal',1);members=1;stages=cell(size(W,1)-1,1);
polygons=parking.PolygonData(c);points=[W(:,1:2);vertcat(polygons.vertices{:})];lower=min(points)-o.padding;upper=max(points)+o.padding;
report=struct('success',false,'code','not_connected','stages',{{}},'nodes',1,'edges',0,'time_s',0);
for stage=1:size(W,1)-1
 graph.q(end+1,:)=W(stage+1,:);goal=size(graph.q,1);S=members;G=goal;joined=false;initialCount=size(graph.q,1);tested=0;samples=0;
 delta=W(stage+1,:)-W(stage,:);delta(3)=atan2(sin(delta(3)),cos(delta(3)));
 mean=W(stage,:)+o.meanFraction*delta;covariance=o.gaussianWeights*norm((1-o.meanFraction)*delta);
 for sample=1:o.stageSampleLimit
  if size(graph.q,1)-initialCount>=o.addedNodesPerStage||toc(clock)>o.kinematicSeconds,break;end
  samples=sample;target=mean+randn(1,3).*sqrt(max(covariance,1e-12));target(3)=atan2(sin(target(3)),cos(target(3)));
  if any(target(1:2)<lower|target(1:2)>upper)||~parking.FootprintClearance(target,c,0),continue;end
  node=0;nearS=nearest(S,target);nearG=nearest(G,target);
  if nearS>0
   [pp,ok,count]=wgrrt.Connect(connection,graph.q(nearS,:),target,c,o);tested=tested+count;
   if ok,graph.q(end+1,:)=target;node=size(graph.q,1);S(end+1)=node;add(nearS,node,pp);end %#ok<AGROW>
  end
  if nearG>0
   [pp,ok,count]=wgrrt.Connect(connection,graph.q(nearG,:),target,c,o);tested=tested+count;
   if ok
    if node==0,graph.q(end+1,:)=target;node=size(graph.q,1);G(end+1)=node;else,joined=true;end %#ok<AGROW>
    add(nearG,node,pp);
   end
  end
 end
 stages{stage}=struct('success',joined,'new_nodes',size(graph.q,1)-initialCount,'samples',samples,'rs_paths_tested',tested, ...
  'mean',mean,'covariance_diagonal',covariance,'time_s',toc(clock));
 if ~joined,report.code='waypoint_connection_failed';break;end
 members=unique([S G]);graph.goal=goal;
 if stage==size(W,1)-1,report.success=true;report.code='waypoints_connected';end
end
report.stages=stages;report.nodes=size(graph.q,1);report.edges=numel(graph.cost);report.time_s=toc(clock);
 function index=nearest(ids,target)
  values=wgrrt.Metric(graph.q(ids,:),target,o);[distance,k]=min(values);
  if distance<=o.neighborRadius,index=ids(k);else,index=0;end
 end
 function add(a,b,pp)
  graph.from(end+1,1)=a;graph.to(end+1,1)=b;graph.cost(end+1,1)=sum(abs(pp(:,1)));graph.primitives{end+1,1}=pp;
 end
end
