function [poses,groups,searchCase,bounds,startId,goalId]=Grid(c,o)
polygons=parking.PolygonData(c);points=[c.task.x0 c.task.y0;c.task.xf c.task.yf];
for j=1:polygons.count,points=[points;polygons.vertices{j}];end %#ok<AGROW>
bounds=[min(points,[],1)-o.padding,max(points,[],1)+o.padding];
[x,y,a]=ndgrid(bounds(1):o.coarseSpacing:bounds(3),bounds(2):o.coarseSpacing:bounds(4),(0:o.coarseAngles-1)*2*pi/o.coarseAngles);
poses=[x(:) y(:) a(:)];
[u,w,a]=ndgrid(-o.fineRadius:o.fineSpacing:o.fineRadius,-o.fineRadius:o.fineSpacing:o.fineRadius,(0:o.fineAngles-1)*2*pi/o.fineAngles);
x=c.task.xf+cos(c.task.thetaf)*u-sin(c.task.thetaf)*w;y=c.task.yf+sin(c.task.thetaf)*u+cos(c.task.thetaf)*w;a=mod(a+c.task.thetaf,2*pi);
poses=[poses;x(:) y(:) a(:);c.task.x0 c.task.y0 mod(c.task.theta0,2*pi);c.task.xf c.task.yf mod(c.task.thetaf,2*pi)];
poses=unique(poses,'rows');searchCase=c;
for j=1:polygons.count
 A=polygons.A{j};b=polygons.b{j}+o.obstacleMargin;P=zeros(size(A));
 for k=1:size(A,1),previous=mod(k-2,size(A,1))+1;P(k,:)=([A(previous,:);A(k,:)]\[b(previous);b(k)])';end
 searchCase.obstacle.obs{j}=struct('x',[P(:,1);P(1,1)]','y',[P(:,2);P(1,2)]');
end
[~,gap]=parking.FootprintClearance(poses,searchCase,0);poses=poses(gap>0,:);
startId=find(hypot(poses(:,1)-c.task.x0,poses(:,2)-c.task.y0)<1e-10&abs(atan2(sin(poses(:,3)-c.task.theta0),cos(poses(:,3)-c.task.theta0)))<1e-10,1);
goalId=find(hypot(poses(:,1)-c.task.xf,poses(:,2)-c.task.yf)<1e-10&abs(atan2(sin(poses(:,3)-c.task.thetaf),cos(poses(:,3)-c.task.thetaf)))<1e-10,1);
[xy,~,which]=unique(poses(:,1:2),'rows');groups=struct('xy',xy,'ids',{accumarray(which,(1:size(poses,1))',[],@(x){x})});
end
