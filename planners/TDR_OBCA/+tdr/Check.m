function check = Check(c,q,lambda,mu,d,o,goalHeading)
v=c.vehicle;N=numel(q.t);h=q.t(2)-q.t(1);p=parking.PolygonData(c);
X=[q.x,q.y,q.theta,q.v];U=[q.phi(1:end-1),q.a(1:end-1)];
F=[q.v(1:end-1).*cos(q.theta(1:end-1)),q.v(1:end-1).*sin(q.theta(1:end-1)), ...
 q.v(1:end-1).*tan(U(:,1))/v.lw,U(:,2)];
defect=max([max(abs(diff(X)-h*F),[],'all');abs(diff(q.phi)-h*q.omega(1:end-1))]);
residual=max([0;abs(X(1,:)-[c.task.x0,c.task.y0,c.task.theta0,0])';abs(q.v(end)); ...
 abs(q.v)-v.vmax;abs(U(:,1))-v.phimax;abs(U(:,2))-v.amax; ...
 abs(q.omega)-v.wmax;abs(q.phi([1 end]));abs(X(end,:)-[c.task.xf,c.task.yf,goalHeading,0])';d(:)+o.strictDistance;-lambda(:);-mu(:)]);
last=cumsum(cellfun(@(x)size(x,1),p.A));first=last-cellfun(@(x)size(x,1),p.A)+1;
g=[v.lw+v.lf;v.lr;v.lb/2;v.lb/2];dualResidual=0;
for i=2:N
 R=[cos(q.theta(i)),-sin(q.theta(i));sin(q.theta(i)),cos(q.theta(i))];
 for j=1:p.count
  l=lambda(i-1,first(j):last(j))';m=reshape(mu(i-1,j,:),4,1);normal=p.A{j}'*l;
  equality=[m(1)-m(2);m(3)-m(4)]+R'*normal;
  distance=-g'*m+(p.A{j}*X(i,1:2)'-p.b{j})'*l+d(i-1,j);
  dualResidual=max([dualResidual;abs(equality);abs(distance);normal'*normal-1]);
 end
end
w=o.weights;
objective=w(1)*sum(X(2:end,:).^2,'all')+w(2)*sum(diff(X).^2,'all') ...
 +(w(3)+w(4))*sum(U.^2,'all')+w(5)*sum((X(end,:)-[c.task.xf,c.task.yf,goalHeading,0]).^2)+w(6)*sum(d,'all');
check=struct('success',max([defect,residual,dualResidual])<o.feasibilityTolerance, ...
 'dynamics_residual',defect,'bound_residual',residual,'dual_residual',dualResidual,'objective',objective);
end
