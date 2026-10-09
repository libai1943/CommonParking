function [route,info]=Explore(c,o)
% Orientation-aware circle search: Algorithm 1 and equation (1).
clock=tic;t=c.task;start=[t.x0 t.y0 t.theta0];goal=[t.xf t.yf t.thetaf];k=c.vehicle.kappa_max;
polygons=parking.PolygonData(c);points=[start(1:2);goal(1:2);vertcat(polygons.vertices{:})];lower=min(points)-o.padding;upper=max(points)+o.padding;
inner=min([c.vehicle.lb/2,c.vehicle.lr,c.vehicle.lw+c.vehicle.lf]);
r0=radius(start(1:2));rg=radius(goal(1:2));route=struct('q',zeros(0,3),'radius',zeros(0,1),'direction',zeros(0,1),'remaining',zeros(0,1));
info=struct('success',false,'code','circle_search_failed','generated',1,'expanded',0,'time_s',0,'bounds',[lower;upper],'inner_radius',inner);
if min(r0,rg)<o.minimumCircle,info.code='endpoint_circle_too_small';return;end
Q=start;R=r0;G=0;parent=0;open=1;priority=osehs.Metric(start,goal,k);closed=[];goalCost=inf;goalParent=0;
for expansion=1:o.maximumCircles
 if isempty(open)||toc(clock)>o.circleSeconds||size(Q,1)>=o.maximumCircles,break;end
 [cost,index]=min(priority);
 if goalCost<cost,break;end
 cur=open(index);open(index)=[];priority(index)=[];
 if ~isempty(closed)&&any(osehs.Metric(Q(closed,:),Q(cur,:),k)<R(closed)-1e-10),continue;end
 closed(end+1,1)=cur; %#ok<AGROW>
 distance=osehs.Metric(Q(cur,:),goal,k);
 if distance<R(cur)+rg-o.overlapFraction*min(R(cur),rg)&&G(cur)+distance<goalCost
  goalCost=G(cur)+distance;goalParent=cur;
 end
 angles=Q(cur,3)+(0:o.circleAngles-1)'*(2*pi/o.circleAngles);
 xy=Q(cur,1:2)+R(cur)*[cos(angles),sin(angles)];
 delta=atan2(sin(angles-Q(cur,3)),cos(angles-Q(cur,3)));delta(delta>pi/2)=delta(delta>pi/2)-pi;delta(delta< -pi/2)=delta(delta< -pi/2)+pi;
 children=[xy,Q(cur,3)+delta];r=radius(xy);
 for j=1:o.circleAngles
  if r(j)<o.minimumCircle||any(xy(j,:)<lower|xy(j,:)>upper),continue;end
  Q(end+1,:)=children(j,:);R(end+1,1)=r(j);G(end+1,1)=G(cur)+osehs.Metric(children(j,:),Q(cur,:),k);parent(end+1,1)=cur; %#ok<AGROW>
  open(end+1)=size(Q,1);priority(end+1)=G(end)+osehs.Metric(children(j,:),goal,k); %#ok<AGROW>
 end
end
info.generated=size(Q,1);info.expanded=numel(closed);info.time_s=toc(clock);
if goalParent==0,return;end
chain=goalParent;while parent(chain(1))>0,chain=[parent(chain(1));chain];end %#ok<AGROW>
route.q=[Q(chain,:);goal];route.radius=[R(chain);rg];route=osehs.Preprocess(route,k);
info.success=true;info.code='circle_path_found';info.distance=goalCost;info.circles=size(route.q,1);
 function r=radius(xy)
  r=min(o.maximumCircle,parking.NearestObstacle(xy,c)-inner);
 end
end
