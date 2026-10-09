function report=TestSE2NMPC()
SetupCommonParking();o=se2mpc.Config();o.intervals=20;o.speedCost=0;o.maxCpuSeconds=40;
c=LoadCase(1);c.obstacle.num_obs=0;c.obstacle.obs={};c.task=struct('x0',0,'y0',0,'theta0',-4*pi,'xf',3,'yf',0,'thetaf',6*pi);
polygons=parking.PolygonData(c);N=o.intervals;h=sqrt(3)/10;t=(0:N)'*h;
speed=h*min((0:N)',N-(0:N)');x=[0;cumsum(speed(1:end-1))*h];
initial=struct('t',t,'x',x,'y',zeros(N+1,1),'theta',zeros(N+1,1),'v',speed,'phi',zeros(N+1,1));
[lambda,mu]=se2mpc.DualSeed(c,[initial.x initial.y initial.theta],polygons);
[q,solver,d]=se2mpc.Solve(c,initial,polygons,lambda,mu,o);
assert(solver.success,solver.message);timeError=abs(q.t(end)-2*sqrt(3));assert(timeError<1e-5);
shifted=d.poses;shifted(:,3)=shifted(:,3)+2*pi*mod((0:N)',4);
check=se2mpc.Check(c,shifted,d.controls,d.h,d.lambda,d.mu,polygons,o);assert(check.success);
assert(abs(check.objective-d.check.objective)<1e-9);
% Same analytic discrete time optimum with reverse motion and opposite heading.
c.task.theta0=pi;c.task.thetaf=-pi;initial.theta(:)=pi;initial.v=-speed;
[qReverse,solverReverse]=se2mpc.Solve(c,initial,polygons,lambda,mu,o);assert(solverReverse.success,solverReverse.message);
assert(abs(qReverse.t(end)-2*sqrt(3))<1e-5&&min(qReverse.v)<-1);
report=struct('passed',true,'analytic_straight_time_error',timeError,'reverse_time_error',abs(qReverse.t(end)-2*sqrt(3)), ...
 'arbitrary_two_pi_shift_residual',check.maximum_equality_error,'forward_native_flag',solver.solve_result_num,'reverse_native_flag',solverReverse.solve_result_num);disp(report);
end
