function [path,info]=GeometricPath(c,options)
% Section III-B2 permits a collision-free geometric planner for waypoints.
% This implementation uses a bidirectional geometric RRT-Connect in SE(2).
clock=tic;t=c.task;start=[t.x0 t.y0 t.theta0];goal=[t.xf t.yf t.thetaf];
info=struct('success',false,'iterations',0,'nodes',2,'time_s',0);path=zeros(0,3);
if wgrrt.GeometricEdgeFree(start,goal,c,options),path=[start;goal];info.success=true;info.time_s=toc(clock);path(:,3)=unwrap(path(:,3));return;end
polygons=parking.PolygonData(c);points=[start(1:2);goal(1:2);vertcat(polygons.vertices{:})];lower=min(points)-options.padding;upper=max(points)+options.padding;
bodyRadius=hypot(max(c.vehicle.lr,c.vehicle.lw+c.vehicle.lf),c.vehicle.lb/2);
A=struct('q',start,'parent',0,'from_start',true);B=struct('q',goal,'parent',0,'from_start',false);
for iteration=1:options.holonomicIterations
 if toc(clock)>options.holonomicSeconds,break;end
 if rand<options.goalBias,sample=B.q(1,:);else,sample=[lower+rand(1,2).*(upper-lower),start(3)-pi+2*pi*rand];end
 [A,added,~]=extend(A,sample,false);
 if added>0
  [B,join,reached]=extend(B,A.q(added,:),true);
  if reached
   a=trace(A,added);b=trace(B,join);
   if A.from_start,path=[a;flipud(b(1:end-1,:))];else,path=[b;flipud(a(1:end-1,:))];end
   path(:,3)=unwrap(path(:,3));info.success=true;break;
  end
 end
 temp=A;A=B;B=temp;
end
info.iterations=iteration;info.nodes=size(A.q,1)+size(B.q,1);info.time_s=toc(clock);
 function [tree,index,reached]=extend(tree,target,greedy)
  angle=atan2(sin(tree.q(:,3)-target(3)),cos(tree.q(:,3)-target(3)));
  metric=hypot(tree.q(:,1)-target(1),tree.q(:,2)-target(2))+bodyRadius*abs(angle);[~,near]=min(metric);index=0;reached=false;
  for expansion=1:500
   q=tree.q(near,:);delta=target-q;delta(3)=atan2(sin(delta(3)),cos(delta(3)));distance=norm(delta(1:2))+bodyRadius*abs(delta(3));
   if distance<1e-10,index=near;reached=true;return;end
   ratio=min(1,options.holonomicStep/distance);next=q+ratio*delta;
   if ~wgrrt.GeometricEdgeFree(q,next,c,options),return;end
   tree.q(end+1,:)=next;tree.parent(end+1,1)=near;index=size(tree.q,1);near=index;
   if ratio==1,reached=true;return;end
   if ~greedy||toc(clock)>options.holonomicSeconds,return;end
  end
 end
end
function points=trace(tree,index)
indices=zeros(0,1);
while index>0,indices(end+1,1)=index;index=tree.parent(index);end %#ok<AGROW>
points=tree.q(flipud(indices),:);
end

