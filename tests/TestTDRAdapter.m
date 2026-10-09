function report=TestTDRAdapter()
% Synthetic turning/rest fixture and negative checks for omitted constraints.
c=LoadCase(1);o=tdr.Config();c.obstacle.num_obs=0;c.obstacle.obs={};
h=.2;u=[1 .4;1 -.4;-1 0;-1 0];z=zeros(5,5);z(1,3)=7*pi;
for i=1:4
 z(i+1,:)=z(i,:)+h*[z(i,4)*cos(z(i,3)),z(i,4)*sin(z(i,3)),z(i,4)*tan(z(i,5))/c.vehicle.lw,u(i,:)];
end
c.task=struct('x0',z(1,1),'y0',z(1,2),'theta0',z(1,3),'xf',z(end,1),'yf',z(end,2),'thetaf',z(end,3)-6*pi);
q=struct('t',(0:4)'*h,'x',z(:,1),'y',z(:,2),'theta',z(:,3),'v',z(:,4),'phi',z(:,5),'a',[u(:,1);0],'omega',[u(:,2);0]);
check=@(p)tdr.Check(c,p,zeros(4,0),zeros(4,0,4),zeros(4,0),o,z(end,3));
good=check(q);assert(good.success);
bad=q;bad.phi(3)=bad.phi(3)+.1;assert(~check(bad).success);
bad=q;bad.phi(end)=.05;assert(~check(bad).success);
bad=q;bad.omega(1)=c.vehicle.wmax+1;assert(~check(bad).success);
bad=q;bad.x(end)=bad.x(end)+.02;assert(~check(bad).success);
report=struct('passed',true,'five_state_euler_residual',good.dynamics_residual,'exact_terminal_and_steering_checks',true);disp(report);
end
