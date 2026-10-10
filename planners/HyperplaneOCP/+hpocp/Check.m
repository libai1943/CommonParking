function report=Check(c,z,u,h,planes,o)
prediction=hpocp.Step(z(1:end-1,:),u,h,c.vehicle.lw);dynamic=max(abs(prediction-z(2:end,:)),[],'all');
t=c.task;boundary=[z(1,:)-[t.x0 t.y0 t.theta0 0 0];z(end,:)-[t.xf t.yf t.thetaf 0 0]];boundary(:,3)=atan2(sin(boundary(:,3)),cos(boundary(:,3)));
v=c.vehicle;excess=[abs(z(:,4))-v.vmax;abs(z(:,5))-v.phimax;abs(u(:,1))-v.amax;abs(u(:,2))-v.wmax;o.minimumTime-h*size(u,1);h*size(u,1)-o.maximumTime];
frames=hpocp.Frames(z(:,1:3),v);polygons=parking.PolygonData(c);minimumNormal=inf;minimumSupport=inf;
for j=1:polygons.count
 p=reshape(planes(:,j,:),[],3);norms=hypot(p(:,1),p(:,2));minimumNormal=min(minimumNormal,min(norms));
 rho=hypot(max(v.lw+v.lf,v.lr),v.lb/2);vv=max(abs(z(1:end-1,4)),abs(z(2:end,4)));pp=max(abs(z(1:end-1,5)),abs(z(2:end,5)));
 aa=abs(u(:,1));ww=abs(u(:,2));kk=tan(pp)/v.lw;
 curvatureBound=aa+vv.^2.*kk+rho*(aa.*kk+vv.*ww./(v.lw*cos(pp).^2)+(vv.*kk).^2);
 reserve=max(rho/8*diff(z(:,3)).^2,h^2/8*curvatureBound)+o.supportTolerance;
 for side=0:1
  body=p(:,1).*frames(1+side:end-1+side,:,1)+p(:,2).*frames(1+side:end-1+side,:,2)-p(:,3)-reserve;
  obstacle=p(:,3)-p(:,1:2)*polygons.vertices{j}';
  excess=[excess;abs(norms.^2-1);-body(:);-obstacle(:)];
  minimumSupport=min(minimumSupport,min([min(body,[],2)./max(norms,realmin);min(obstacle,[],2)./max(norms,realmin)]));
 end
end
inequality=max([0;excess]);endpoint=max(abs(boundary),[],'all');objective=h*sum(o.timeWeight+o.accelerationWeight*u(:,1).^2+o.steeringRateWeight*u(:,2).^2);
report=struct('success',dynamic<=o.feasibilityTolerance&&endpoint<=o.feasibilityTolerance&&inequality<=o.feasibilityTolerance, ...
 'dynamics_error',dynamic,'boundary_error',endpoint,'inequality_violation',inequality,'minimum_normal',minimumNormal, ...
 'minimum_normalized_support_m',minimumSupport,'objective',objective,'tf',h*size(u,1));
end
