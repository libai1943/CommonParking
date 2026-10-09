function report=TestIndirectOCP()
% Independent derivatives, PMP transversality, output and cusp checks.
SetupCommonParking();c=LoadCase(3);o=indpark.Config();ctx=indpark.Context(c,o);rng(2023165);y=randn(10,8);y(4,:)=.3*y(4,:);y(5,:)=.1*y(5,:);reference=randn(3,8);jac=0;ham=0;
for mode={'tracking','obstacle','blend'}
 ctx.mode=mode{1};ctx.blend=.37;[~,D,~,HG]=indpark.Kernel(y,ctx,reference);
 for j=1:10
  perturb=zeros(size(y));perturb(j,:)=1i*1e-20;[f,~,H]=indpark.Kernel(y+perturb,ctx,reference);
  jac=max(jac,max(abs(imag(f)/1e-20-squeeze(D(:,j,:))),[],'all')/max(1,max(abs(D),[],'all')));
  ham=max(ham,max(abs(imag(H)/1e-20-HG(j,:)),[],'all')/max(1,max(abs(HG),[],'all')));
 end
end
assert(jac<1e-10&&ham<1e-10);
lambda=randn(2,50)*3;[u]=indpark.Control(lambda,[c.vehicle.amax;c.vehicle.wmax],.1);[~,g]=indpark.Barrier(u,[c.vehicle.amax;c.vehicle.wmax],.1);stationarity=max(abs(g+lambda),[],'all');assert(stationarity<1e-10);
o.nodes=81;ctx=indpark.Context(c,o);ctx.start=[0;0;0;0];ctx.goal=[4;0;0;0];ctx.rectangles=zeros(0,5);s=linspace(0,1,o.nodes);T=5;q=[2*(1-cos(pi*s));zeros(2,o.nodes);2*pi/T*sin(pi*s);zeros(1,o.nodes)];ctx.reference=(q(1:3,1:end-1)+q(1:3,2:end))/2;Y=[q;zeros(size(q))];z=[Y(:);log(T)];
[~,J]=indpark.Residual(z,ctx);v=randn(size(z));fd=(indpark.Residual(z+1e-6*v,ctx)-indpark.Residual(z-1e-6*v,ctx))/2e-6;fullJacobian=norm(fd-J*v,inf)/max(1,norm(J*v,inf));assert(fullJacobian<1e-6);
[z,info]=indpark.Solve(z,ctx);assert(info.success);[z,ctx,~,success]=indpark.ObstacleContinuation(z,ctx);assert(success);[z,ctx,~,success]=indpark.Regularize(z,ctx);assert(success);[trajectory,native]=indpark.Output(z,ctx);assert(abs(trajectory.t(end)-4)<.1);assert(max(abs(trajectory.y))<1e-8&&max(abs(trajectory.phi))<1e-8);
% A synthetic piecewise-linear native speed changes gear twice, off the grid.
ctx.options.nodes=4;ctx.reference=zeros(3,3);Y=zeros(10,4);Y(1,:)=[0 .1 .2 .3];Y(4,:)=[.2 -.7 .4 0];zz=[Y(:);log(3)];[p,n]=indpark.Output(zz,ctx);assert(numel(n.cusp_indices)==2&&all(p.v(n.cusp_indices)==0));assert(max(abs(p.t(n.cusp_indices)-[.2/.9;1+.7/1.1]))<1e-14);assert(all(diff(p.t)>0));
report=struct('canonical_jacobian_relative_error',jac,'hamiltonian_gradient_relative_error',ham,'control_stationarity_error',stationarity,'boundary_value_jacobian_relative_error',fullJacobian,'straight_rest_to_rest_time_s',trajectory.t(end),'straight_residual',native.residual,'exact_off_grid_cusps',true);disp(report);
end
