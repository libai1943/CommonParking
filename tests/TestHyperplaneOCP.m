function report=TestHyperplaneOCP()
c=LoadCase(1);v=c.vehicle;o=hpocp.Config();
z=[1 2 .7 .8 -.2];u=[.3 .1];h=.1;coarse=hpocp.Step(z,u,h,v.lw);fine=z;
for k=1:100,fine=hpocp.Step(fine,u,h/100,v.lw);end
error=max(abs(coarse-fine));assert(error<1e-8);
c.task.x0=0;c.task.y0=0;c.task.theta0=.3;c.task.xf=4;c.task.yf=2;c.task.thetaf=.3+pi/2;
c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[20 22 22 20 20],'y',[20 20 22 22 20])};o.intervals=10;
[z,u,h,p]=hpocp.Initialize(c,o);assert(all(z(:,4:5)==0,'all')&&all(u==0,'all'));assert(h==5);
normal=reshape(p(:,1,1:2),[],2);centroid=[21 21];assert(max(abs(vecnorm(normal,2,2)-1))<1e-12);
direction=z(:,1:2)-centroid;assert(max(abs(normal(:,1).*direction(:,2)-normal(:,2).*direction(:,1)))<1e-12);
point=.75*z(:,1:2)+.25*centroid;assert(max(abs(sum(normal.*point,2)-p(:,1,3)))<1e-12);
frame=hpocp.Frames([0 0 0],v);xy=[frame(:,:,1)',frame(:,:,2)'];assert(abs(polyarea(xy(:,1),xy(:,2))-v.length*v.lb)<1e-12);
report=struct('passed',true,'rk4_refinement_error',error,'geometric_plane_initialization',true,'linear_zero_control_initialization',true);disp(report);
end
