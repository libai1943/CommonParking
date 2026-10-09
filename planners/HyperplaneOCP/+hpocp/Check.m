function report=Check(c,z,u,h,planes,o)
prediction=hpocp.Step(z(1:end-1,:),u,h,c.vehicle.lw);dynamic=max(abs(prediction-z(2:end,:)),[],'all');
t=c.task;boundary=[z(1,:)-[t.x0 t.y0 t.theta0 0 0];z(end,:)-[t.xf t.yf t.thetaf 0 0]];boundary(:,3)=atan2(sin(boundary(:,3)),cos(boundary(:,3)));
v=c.vehicle;excess=[abs(z(:,4))-v.vmax;abs(z(:,5))-v.phimax;abs(u(:,1))-v.amax;abs(u(:,2))-v.wmax;o.minimumTime-h*size(u,1);h*size(u,1)-o.maximumTime];
frames=hpocp.Frames(z(:,1:3),v);polygons=parking.PolygonData(c);minimumNormal=inf;minimumSupport=inf;
for j=1:polygons.count
 p=reshape(planes(:,j,:),[],3);norms=hypot(p(:,1),p(:,2));minimumNormal=min(minimumNormal,min(norms));
 body=p(:,1).*frames(:,:,1)+p(:,2).*frames(:,:,2)-p(:,3);obstacle=p(:,3)-p(:,1:2)*polygons.vertices{j}';
 excess=[excess;o.normalMinimum^2-norms.^2;-body(:);-obstacle(:)]; %#ok<AGROW>
 minimumSupport=min(minimumSupport,min([min(body,[],2)./max(norms,realmin);min(obstacle,[],2)./max(norms,realmin)]));
end
inequality=max([0;excess]);endpoint=max(abs(boundary),[],'all');objective=h*sum(o.timeWeight+o.accelerationWeight*u(:,1).^2+o.steeringRateWeight*u(:,2).^2);
report=struct('success',dynamic<=o.feasibilityTolerance&&endpoint<=o.feasibilityTolerance&&inequality<=o.feasibilityTolerance, ...
 'dynamics_error',dynamic,'boundary_error',endpoint,'inequality_violation',inequality,'minimum_normal',minimumNormal, ...
 'minimum_normalized_support_m',minimumSupport,'objective',objective,'tf',h*size(u,1));
end
