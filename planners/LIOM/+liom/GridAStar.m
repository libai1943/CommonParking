function [path,info] = GridAStar(c,start,goal,ds)
% Eight-connected, obstacle-aware 2-D A* for the paper's failed-HA extension.
allxy=[start;goal];for j=1:c.obstacle.num_obs,o=c.obstacle.obs{j};allxy=[allxy;o.x' o.y'];end %#ok<AGROW>
lower=min(allxy)-6;upper=max(allxy)+6;dims=ceil((upper-lower)/ds)+1;
[X,Y]=ndgrid(lower(1)+(0:dims(1)-1)*ds,lower(2)+(0:dims(2)-1)*ds);
p=parking.PolygonData(c);points=[X(:) Y(:)];
% Occupied square cells prevent diagonal corner cutting through polygons.
b=[points(:,1)-ds/2 points(:,1)+ds/2 points(:,2)-ds/2 points(:,2)+ds/2];
blocked=cp.BoxObstacleDistance(b,p)<=0;
si=round((start-lower)/ds)+1;gi=round((goal-lower)/ds)+1;
s=si(1)+(si(2)-1)*dims(1);g=gi(1)+(gi(2)-1)*dims(1);
path=[];info=struct('success',false,'expanded',0,'resolution',ds);
if blocked(s)||blocked(g),return;end
cost=inf(prod(dims),1);cost(s)=0;parent=zeros(size(cost));closed=false(size(cost));
open=s;priority=norm(start-goal);offset=[-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
while ~isempty(open)
    [~,k]=min(priority);id=open(k);open(k)=[];priority(k)=[];
    if closed(id),continue;end
    closed(id)=true;info.expanded=info.expanded+1;if id==g,break;end
    ij=[mod(id-1,dims(1))+1 floor((id-1)/dims(1))+1];
    for k=1:8
        n=ij+offset(k,:);if any(n<1)||any(n>dims),continue;end
        child=n(1)+(n(2)-1)*dims(1);if blocked(child)||closed(child),continue;end
        if all(offset(k,:)~=0)
            cross1=n(1)+(ij(2)-1)*dims(1);cross2=ij(1)+(n(2)-1)*dims(1);
            if blocked(cross1)||blocked(cross2),continue;end
        end
        candidate=cost(id)+ds*norm(offset(k,:));if candidate>=cost(child),continue;end
        cost(child)=candidate;parent(child)=id;open(end+1)=child; %#ok<AGROW>
        priority(end+1)=candidate+norm(points(child,:)-goal); %#ok<AGROW>
    end
end
if ~closed(g),return;end
ids=g;while ids(1)~=s,ids=[parent(ids(1));ids];end %#ok<AGROW>
path=[start;points(ids,:);goal];path([false;vecnorm(diff(path),2,2)<1e-10],:)=[];
% Verify the two off-grid endpoint connectors as well as grid edges.
for j=1:size(path,1)-1
    q=path(j,:)+linspace(0,1,max(2,ceil(norm(path(j+1,:)-path(j,:))/.02)+1))'.*(path(j+1,:)-path(j,:));
    if any(cp.BoxObstacleDistance([q(:,1) q(:,1) q(:,2) q(:,2)],p)<=0),path=[];return;end
end
info.success=true;
end
