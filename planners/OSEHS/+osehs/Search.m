function [pp,info]=Search(route,c,o,factor,seconds)
% Algorithm 2: circle-guided primitives, circle-local continuous clustering.
clock=tic;start=[c.task.x0 c.task.y0 c.task.theta0];goal=[c.task.xf c.task.yf c.task.thetaf];k=c.vehicle.kappa_max;
rs=reedsSheppConnection('MinTurningRadius',1/k,'ForwardCost',1,'ReverseCost',1);
polygons=parking.PolygonData(c);points=[route.q(:,1:2);vertcat(polygons.vertices{:})];lower=min(points)-o.padding;upper=max(points)+o.padding;
Q=zeros(o.maximumNodes,3);G=inf(o.maximumNodes,1);parent=zeros(o.maximumNodes,1);primitive=zeros(o.maximumNodes,2);gear=zeros(o.maximumNodes,1);
Q(1,:)=start;G(1)=0;count=1;expanded=0;open=1;[~,priority]=osehs.Map(start,route,k);closed=cell(size(route.q,1),1);
goalCost=inf;finish=0;tail=zeros(0,2);pp=zeros(0,2);queries=0;minH=inf;closest=start;
while ~isempty(open)&&expanded<o.maximumExpanded&&toc(clock)<seconds&&count+6<o.maximumNodes
 [f,index]=min(priority);if goalCost<f,break;end
 cur=open(index);open(index)=[];priority(index)=[];q=Q(cur,:);[circle,h]=osehs.Map(q,route,k);bucket=closed{circle};
 if h<minH,minH=h;closest=q;end
 resolution=o.clusterFactor*factor*route.radius(circle);
 if ~isempty(bucket)
  angle=abs(atan2(sin(Q(bucket,3)-q(3)),cos(Q(bucket,3)-q(3))));
  similar=vecnorm(Q(bucket,1:2)-q(1:2),2,2)<resolution & angle<resolution*k & gear(bucket)==gear(cur);
  if any(similar&G(bucket)<=G(cur)+1e-10),continue;end
 end
 closed{circle}(end+1,1)=cur;expanded=expanded+1;step=max(o.minimumStep,factor*route.radius(circle));
 if h<o.goalRange
  [paths,costs]=connect(rs,q,goal,'PathSegments','all');[~,order]=sort(costs(:));
  for candidate=order'
   if ~isfinite(costs(candidate)),continue;end
   candidatePP=fromRS(paths{candidate},k);value=G(cur)+pathCost(q,candidatePP,gear(cur));
   if value>=goalCost,continue;end
   queries=queries+1;
   if osehs.ArcsFree(q,candidatePP,c),goalCost=value;finish=cur;tail=candidatePP;end
  end
 end
 for direction=[1 -1]
  for curvature=[-k 0 k]
   travel=direction*step;next=parking.IntegratePrimitive(q,travel,curvature);
   if any(next(1:2)<lower|next(1:2)>upper),continue;end
   value=G(cur)+edgeCost(step,direction,gear(cur),circle);queries=queries+1;
   if value>=goalCost||~osehs.ArcsFree(q,[travel curvature],c),continue;end
   count=count+1;Q(count,:)=next;G(count)=value;parent(count)=cur;primitive(count,:)=[travel curvature];gear(count)=direction;
   [~,heuristic]=osehs.Map(next,route,k);open(end+1)=count;priority(end+1)=value+heuristic; %#ok<AGROW>
  end
 end
end
if finish>0
 cur=finish;while parent(cur)>0,pp=[primitive(cur,:);pp];cur=parent(cur);end %#ok<AGROW>
 pp=[pp;tail];
end
info=struct('success',finish>0,'expanded',expanded,'generated',count,'collision_queries',queries,'time_s',toc(clock),'step_factor',factor,'objective',goalCost,'minimum_route_heuristic',minH,'closest_route_pose',closest);
 function value=edgeCost(length,direction,previous,circle)
  preferred=route.direction(circle);weight=1;if preferred~=0&&preferred~=direction,weight=o.wrongDirectionFactor;end
  penalty=o.cuspPenalty;if preferred==0,penalty=o.bidirectionalCuspPenalty;end
  value=length*weight+penalty*(previous~=0&&previous~=direction);
 end
 function value=pathCost(q,arcs,previous)
  value=0;
  for m=1:size(arcs,1)
   [arcCircle,~]=osehs.Map(q,route,k);arcDirection=sign(arcs(m,1));value=value+edgeCost(abs(arcs(m,1)),arcDirection,previous,arcCircle);
   q=parking.IntegratePrimitive(q,arcs(m,1),arcs(m,2));previous=arcDirection;
  end
 end
end
function pp=fromRS(path,k)
curvature=zeros(numel(path.MotionLengths),1);
for j=1:numel(curvature),if strcmp(path.MotionTypes{j},'L'),curvature(j)=k;elseif strcmp(path.MotionTypes{j},'R'),curvature(j)=-k;end,end
pp=[path.MotionLengths(:).*path.MotionDirections(:),curvature];pp(abs(pp(:,1))<1e-10,:)=[];
end
