function check=Check(c,q,lambda,mu,polygons,referenceTime,o)
% Independently reconstruct the five-state RK2 defects and distance duals.
v=c.vehicle;z=[q.x q.y q.theta q.v q.phi];h=diff(q.t);N=size(z,1);
u=[q.a(1:end-1) q.omega(1:end-1)];left=z(1:end-1,:);
f=@(p)[p(:,4).*cos(p(:,3)),p(:,4).*sin(p(:,3)),p(:,4).*tan(p(:,5))/v.lw,u];
mid=left+.5*h.*f(left);dynamic=diff(z)-h.*f(mid);
task=c.task;endpoint=z([1 end],:)-[task.x0 task.y0 task.theta0 0 0;task.xf task.yf task.thetaf 0 0];
endpoint(:,3)=atan2(sin(endpoint(:,3)),cos(endpoint(:,3)));
eq=max([abs(dynamic(:));abs(endpoint(:));abs(h-mean(h))]);
ineq=max([0;abs(q.v)-v.vmax;abs(q.phi)-v.phimax;abs(u(:,1))-v.amax;abs(u(:,2))-v.wmax; ...
 o.timeScale(1)*referenceTime-q.t(end);q.t(end)-o.timeScale(2)*referenceTime]);
g=[v.lw+v.lf;v.lr;v.lb/2;v.lb/2];offset=0;clearance=inf;dualError=0;normalError=0;
for j=1:polygons.count
 A=polygons.A{j};b=polygons.b{j};indices=offset+(1:size(A,1));offset=indices(end);
 l=lambda(:,indices);m=reshape(mu(:,j,:),N,4);normal=l*A;
 local=[cos(q.theta).*normal(:,1)+sin(q.theta).*normal(:,2),-sin(q.theta).*normal(:,1)+cos(q.theta).*normal(:,2)];
 residual=[m(:,1)-m(:,2),m(:,3)-m(:,4)]+local;
 dualError=max(dualError,max(abs(residual),[],'all'));normalError=max(normalError,max(abs(sum(normal.^2,2)-1)));
 distance=-m*g+sum(([q.x q.y]*A'-b').*l,2);clearance=min(clearance,min(distance));
 ineq=max([ineq;-l(:);-m(:);o.clearance-distance]);
end
eq=max([eq dualError normalError]);controls=[q.phi(1:end-1) u(:,1)];rates=[controls(1,:);diff(controls)]/mean(h);
objective=o.timeWeightPerSecond*q.t(end)+sum(sum(controls.^2.*o.controlWeights))+sum(sum(rates.^2.*o.rateWeights));
check=struct('success',eq<=1e-6&&ineq<=1e-6,'maximum_equality_error',eq,'maximum_inequality_violation',ineq, ...
 'maximum_dynamics_error',max(abs(dynamic),[],'all'),'maximum_boundary_error',max(abs(endpoint),[],'all'), ...
 'maximum_dual_error',dualError,'maximum_normal_error',normalError,'minimum_dual_clearance',clearance,'objective',objective);
end
