function [z,u,h,planes]=Initialize(c,o)
% Linear single-car state guess (VII-A-5) and geometric plane guess (VI-B).
t=c.task;fraction=linspace(0,1,o.intervals+1)';dtheta=atan2(sin(t.thetaf-t.theta0),cos(t.thetaf-t.theta0));
z=[t.x0+(t.xf-t.x0)*fraction,t.y0+(t.yf-t.y0)*fraction,t.theta0+dtheta*fraction,zeros(o.intervals+1,2)];
u=zeros(o.intervals,2);h=o.initialTime/o.intervals;polygons=parking.PolygonData(c);planes=zeros(o.intervals+1,polygons.count,3);
for j=1:polygons.count
 P=polygons.vertices{j};Q=P([2:end 1],:);cross=P(:,1).*Q(:,2)-Q(:,1).*P(:,2);centroid=sum((P+Q).*cross,1)/(3*sum(cross));
 normal=z(:,1:2)-centroid;zero=vecnorm(normal,2,2)<1e-12;normal(zero,:)=repmat([1 0],nnz(zero),1);normal=normal./vecnorm(normal,2,2);
 point=o.hyperplaneFraction*z(:,1:2)+(1-o.hyperplaneFraction)*centroid;
 planes(:,j,1:2)=reshape(normal,[],1,2);planes(:,j,3)=sum(normal.*point,2);
end
end
