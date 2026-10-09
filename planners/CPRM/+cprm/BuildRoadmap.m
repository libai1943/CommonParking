function map=BuildRoadmap(c,o)
clock=tic;polygons=parking.PolygonData(c);p=[c.task.x0 c.task.y0;c.task.xf c.task.yf;vertcat(polygons.vertices{:})];lower=min(p)-o.padding;upper=max(p)+o.padding;
radius=min(c.vehicle.lb,c.vehicle.length)/4;points=zeros(0,2);trials=0;
while size(points,1)<o.controlNodes&&trials<o.controlTrials
 batch=lower+rand(200,2).*(upper-lower);trials=trials+size(batch,1);free=cprm.DiskFree(batch,c,radius);points=[points;batch(free,:)]; %#ok<AGROW>
end
points=points(1:min(o.controlNodes,size(points,1)),:);count=size(points,1);distance=hypot(points(:,1)-points(:,1)',points(:,2)-points(:,2)');distance(1:count+1:end)=inf;
[~,nearest]=sort(distance,2);k=min(o.neighbors,count-1);edges=unique(sort([repelem((1:count)',k),reshape(nearest(:,1:k)',[],1)],2),'rows');
midpoint=(points(edges(:,1),:)+points(edges(:,2),:))/2;free=cprm.DiskFree(midpoint,c,radius);edges=edges(free,:);midpoint=midpoint(free,:);
vector=points(edges(:,2),:)-points(edges(:,1),:);heading=atan2(vector(:,2),vector(:,1));q=[midpoint heading;midpoint heading+pi];
[~,gap]=parking.FootprintClearance(q,c,0);valid=gap>o.collisionClearance;number=size(edges,1);adj=cell(count,1);
for j=1:number,adj{edges(j,1)}(end+1)=j;adj{edges(j,2)}(end+1)=j;end
ends=zeros(0,2);weight=zeros(0,1);curvature=zeros(0,1);via=zeros(0,1);primitives=cell(0,1);
for cp=1:count
 incident=adj{cp};
 for ia=1:numel(incident)
  for ib=ia+1:numel(incident)
   a=incident(ia);b=incident(ib);[pp,first,last]=cprm.Fillet(midpoint(a,:),midpoint(b,:),points(cp,:),1);
   if isempty(pp),continue;end
   maxK=max(abs(pp(:,2)));if maxK>o.coarseCurvature,continue;end
   left=a+number*(abs(atan2(sin(first(3)-heading(a)),cos(first(3)-heading(a))))>pi/2);
   right=b+number*(abs(atan2(sin(last(3)-heading(b)),cos(last(3)-heading(b))))>pi/2);
   for orientation=1:2
    if orientation==2,left=1+mod(left-1+number,2*number);right=1+mod(right-1+number,2*number);pp=-pp;end
    if ~valid(left)||~valid(right),continue;end
    ends(end+1,:)=[left right];weight(end+1,1)=sum(abs(pp(:,1)));curvature(end+1,1)=maxK;via(end+1,1)=cp;primitives{end+1,1}=pp; %#ok<AGROW>
   end
  end
 end
end
map=struct('control_points',points,'control_edges',edges,'nodes',q,'valid_nodes',valid,'ends',ends,'weight',weight,'curvature',curvature,'via',via,'primitives',{primitives},'time_s',toc(clock),'control_trials',trials,'control_disk_radius',radius);
end
