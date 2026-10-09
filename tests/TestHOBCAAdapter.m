function report=TestHOBCAAdapter()
% A constructed RK2 trajectory exercises steering as a continuous state.
c=LoadCase(1);o=hobca.Config();h=.2;u=[1 .4;1 -.4;-1 0;-1 0];z=zeros(5,5);z(1,3)=7*pi;
for i=1:4
 flow=@(p)[p(4)*cos(p(3)),p(4)*sin(p(3)),p(4)*tan(p(5))/c.vehicle.lw,u(i,:)];
 z(i+1,:)=z(i,:)+h*flow(z(i,:)+h/2*flow(z(i,:)));
end
c.task=struct('x0',z(1,1),'y0',z(1,2),'theta0',pi,'xf',z(end,1),'yf',z(end,2),'thetaf',z(end,3)-6*pi);
q=struct('t',(0:4)'*h,'x',z(:,1),'y',z(:,2),'theta',z(:,3),'v',z(:,4),'phi',z(:,5),'a',[u(:,1);0],'omega',[u(:,2);0]);
p=struct('count',0);good=hobca.Check(c,q,zeros(5,0),zeros(5,0,4),p,.8,o);assert(good.success);
bad=q;bad.phi(3)=bad.phi(3)+.1;assert(~hobca.Check(c,bad,zeros(5,0),zeros(5,0,4),p,.8,o).success);
bad=q;bad.phi(end)=.05;assert(~hobca.Check(c,bad,zeros(5,0),zeros(5,0,4),p,.8,o).success);
bad=q;bad.omega(1)=c.vehicle.wmax+1;assert(~hobca.Check(c,bad,zeros(5,0),zeros(5,0,4),p,.8,o).success);
report=struct('passed',true,'rk2_defect',good.maximum_dynamics_error,'wrapped_terminal_checked',true,'steering_state_and_rate_checked',true);disp(report);
end
