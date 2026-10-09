function check=Check(c,p,u,h,lambda,mu,polygons,o)
N=size(u,1);v=c.vehicle;angle=atan2(sin(diff(p(:,3))),cos(diff(p(:,3))));
dynamic=[diff(p(:,1))/h-.5*u(:,1).*(cos(p(1:end-1,3))+cos(p(2:end,3))); ...
 diff(p(:,2))/h-.5*u(:,1).*(sin(p(1:end-1,3))+sin(p(2:end,3)));angle/h-u(:,1).*tan(u(:,2))/v.lw];
rates=[u(1,:)/o.previousDt;diff(u)/h;-u(end,:)/h];
excess=[abs(u(:,1))-v.vmax;abs(u(:,2))-v.phimax;abs(rates(:,1))-v.amax;abs(rates(:,2))-v.wmax;o.minimumDt-h];
task=c.task;endpoint=p([1 end],:)-[task.x0 task.y0 task.theta0;task.xf task.yf task.thetaf];endpoint(:,3)=atan2(sin(endpoint(:,3)),cos(endpoint(:,3)));
eq=max([abs(dynamic);abs(endpoint(:));abs(u(1,1))]);ineq=max([0;excess]);clearance=inf;normExcess=0;dualError=0;
g=[v.lw+v.lf;v.lr;v.lb/2;v.lb/2];offset=0;
for j=1:polygons.count
 A=polygons.A{j};b=polygons.b{j};indices=offset+(1:size(A,1));offset=indices(end);l=lambda(:,indices);m=squeeze(mu(:,j,:));normal=l*A;
 normExcess=max(normExcess,max(sum(normal.^2,2)-1));
 local=[cos(p(:,3)).*normal(:,1)+sin(p(:,3)).*normal(:,2),-sin(p(:,3)).*normal(:,1)+cos(p(:,3)).*normal(:,2)];
 residual=[m(:,1)-m(:,2),m(:,3)-m(:,4)]+local;dualError=max(dualError,max(abs(residual),[],'all'));
 distance=-m*g+sum((p(:,1:2)*A'-b').*l,2);clearance=min(clearance,min(distance));
 ineq=max([ineq; -l(:);-m(:);o.clearance-distance;sum(normal.^2,2)-1]);
end
eq=max(eq,dualError);check=struct('maximum_equality_error',eq,'maximum_inequality_violation',ineq, ...
 'maximum_dynamics_error',max(abs(dynamic)),'minimum_dual_clearance',clearance,'maximum_dual_normal_excess',max(0,normExcess), ...
 'objective',h*sum(1+o.speedCost*u(:,1).^2),'final_time',N*h,'maximum_control_rate',max(abs(rates),[],1), ...
 'success',eq<=o.feasibilityTolerance&&ineq<=o.feasibilityTolerance);
end
