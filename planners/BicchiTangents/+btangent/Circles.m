function [circles,endpointCircles]=Circles(c)
% Figure 5 shifts centers INTO the polygon; the outer arc grazes its vertex.
v=c.vehicle;R=1/v.kappa_max;h=v.lb/2;polygons=parking.PolygonData(c);circles=[];
for j=1:numel(polygons.vertices)
 p=polygons.vertices{j};signedArea=sum(p(:,1).*p([2:end 1],2)-p(:,2).*p([2:end 1],1));orientation=sign(signedArea);
 for k=1:size(p,1)
  if R<=h,circles(end+1,:)=[p(k,:) h];continue;end %#ok<AGROW>
  before=p(k,:)-p(mod(k-2,size(p,1))+1,:);after=p(mod(k,size(p,1))+1,:)-p(k,:);
  n1=orientation*[-before(2),before(1)]/norm(before);n2=orientation*[-after(2),after(1)]/norm(after);bisector=(n1+n2)/norm(n1+n2);
  circles=[circles;p(k,:)+(R-h)*[bisector;n1;n2],R*ones(3,1)]; %#ok<AGROW>
 end
end
t=c.task;tasks=[t.x0 t.y0 t.theta0;t.xf t.yf t.thetaf];endpointCircles=zeros(2,2);
for j=1:2
 for side=[1 -1]
  centre=tasks(j,1:2)+side*R*[-sin(tasks(j,3)),cos(tasks(j,3))];circles(end+1,:)=[centre R];endpointCircles(j,1+(side<0))=size(circles,1); %#ok<AGROW>
 end
end
% Coincident circles carry the same geometric support and are merged.
[~,keep,map]=unique(round(circles/1e-9),'rows','stable');circles=circles(keep,:);endpointCircles=map(endpointCircles);
end
