function check=Check(q,lambda,keys,c,reference,radius,o,goalHeading)
v=c.vehicle;n=numel(q.x);h=q.t(end)/(n-1);a=q.a(1:end-1);w=q.omega(1:end-1);
model=[diff(q.x)-h*q.v(2:end).*cos(q.theta(2:end)),diff(q.y)-h*q.v(2:end).*sin(q.theta(2:end)), ...
 diff(q.theta)-h*q.v(2:end).*tan(q.phi(2:end))/v.lw,diff(q.v)-h*a,diff(q.phi)-h*w];
boundary=[q.x(1)-c.task.x0;q.y(1)-c.task.y0;q.theta(1)-c.task.theta0;q.v(1);q.phi(1);q.x(end)-c.task.xf;q.y(end)-c.task.yf;q.theta(end)-goalHeading;q.v(end);q.phi(end)];
inequality=[abs(q.v)-v.vmax;abs(q.phi)-v.phimax;abs(a)-v.amax;abs(w)-v.wmax;abs(q.x(2:end-1)-reference.x(2:end-1))-radius;abs(q.y(2:end-1)-reference.y(2:end-1))-radius; ...
 min(c.task.theta0,goalHeading)-pi-q.theta;q.theta-max(c.task.theta0,goalHeading)-pi;o.minimumTime-q.t(end);q.t(end)-o.maximumTime];
polys=parking.PolygonData(c);body=[v.lw+v.lf,v.lb/2;v.lw+v.lf,-v.lb/2;-v.lr,-v.lb/2;-v.lr,v.lb/2];G=[1 0;0 1;-1 0;0 -1];g=[v.lw+v.lf;v.lb/2;v.lr;v.lb/2];minDistance=inf;
for m=1:size(keys,1)
 k=keys(m,1);i=keys(m,3);j=keys(m,4);R=[cos(q.theta(k)) -sin(q.theta(k));sin(q.theta(k)) cos(q.theta(k))];t=[q.x(k);q.y(k)];
 if keys(m,2)==1,p=R*body(j,:)'+t;A=polys.A{i};b=polys.b{i};else,p=R'*(polys.vertices{i}(j,:)'-t);A=G;b=g;end
 l=lambda{m};distance=(A*p-b)'*l;minDistance=min(minDistance,distance);inequality=[inequality;o.dualDistance-distance;norm(A'*l)^2-1;-l]; %#ok<AGROW>
end
residual=max([abs(model(:));abs(boundary);inequality;0]);objective=o.weights(1)*q.t(end)+h*sum(o.weights(2)*(a.^2+q.v(1:end-1).^2.*w.^2)+o.weights(3)*q.phi(1:end-1).^2);
check=struct('success',residual<=o.feasibilityTolerance,'maximum_violation',residual,'implicit_euler_defect',max(abs(model),[],'all'),'boundary_error',max(abs(boundary)),'minimum_dual_separation',minDistance,'objective',objective);
end
