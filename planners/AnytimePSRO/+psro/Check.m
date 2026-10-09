function report=Check(c,q,h,data,reference,o)
z=[q.x q.y q.theta q.v q.phi];u=[q.a q.omega];v=c.vehicle;
flow=[q.v.*cos(q.theta),q.v.*sin(q.theta),q.v.*tan(q.phi)/v.lw,q.a,q.omega];
residual=z(2:end,:)-z(1:end-1,:)-h.*flow(1:end-1,:);dynamics=max(abs(residual),[],'all');
t=c.task;boundary=[z(1,:)-[t.x0 t.y0 t.theta0 0 0];z(end,:)-[t.xf t.yf t.thetaf 0 0]];boundary(:,3)=atan2(sin(boundary(:,3)),cos(boundary(:,3)));
endpoint=max(abs([boundary(:);u(1,:)';u(end,:)']));
violation=max([0;abs(q.v)-v.vmax;abs(q.phi)-v.phimax;abs(q.a)-v.amax;abs(q.omega)-v.wmax;o.minimumStep-h;h-o.maximumStep; ...
 abs(q.x-reference.x)-data.position_radius;abs(q.y-reference.y)-data.position_radius;abs(q.theta-reference.theta)-data.angle_radius]);
E=data.ego_planes;idx=E(:,1);px=q.x(idx)+cos(q.theta(idx)).*E(:,2)-sin(q.theta(idx)).*E(:,3);py=q.y(idx)+sin(q.theta(idx)).*E(:,2)+cos(q.theta(idx)).*E(:,3);
violation=max([violation;E(:,6)-E(:,4).*px-E(:,5).*py]);
O=data.obstacle_planes;idx=O(:,1);dx=O(:,2)-q.x(idx);dy=O(:,3)-q.y(idx);px=cos(q.theta(idx)).*dx+sin(q.theta(idx)).*dy;py=-sin(q.theta(idx)).*dx+cos(q.theta(idx)).*dy;
violation=max([violation;O(:,6)-O(:,4).*px-O(:,5).*py]);
objective=sum(h.*(1+o.accelerationWeight*q.a(1:end-1).^2+o.steeringRateWeight*q.omega(1:end-1).^2));
report=struct('success',max([dynamics endpoint violation])<=o.feasibilityTolerance,'dynamics_residual',dynamics,'boundary_residual',endpoint,'maximum_inequality_violation',violation,'objective',objective);
end
