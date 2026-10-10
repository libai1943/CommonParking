function report=TestHyperplaneOCP()
SetupCommonParking();c=LoadCase(1);v=c.vehicle;o=hpocp.Config();
z=[1 2 .7 .8 -.2];u=[.3 .1];h=.1;coarse=hpocp.Step(z,u,h,v.lw);fine=z;
for k=1:100,fine=hpocp.Step(fine,u,h/100,v.lw);end
error=max(abs(coarse-fine));assert(error<1e-8);
frame=hpocp.Frames([0 0 0],v);xy=[frame(:,:,1)',frame(:,:,2)'];assert(abs(polyarea(xy(:,1),xy(:,2))-v.length*v.lb)<1e-12);
% A real interval collision missed by two separated endpoint rectangles.
rho=hypot(v.lw+v.lf,v.lb/2);middle=-atan2(v.lb/2,v.lw+v.lf);angles=middle+[-.3;.3];
ends=hpocp.Frames([zeros(2,2),angles],v);center=hpocp.Frames([0 0 middle],v);wall=rho-.02;
assert(max(ends(:,:,1),[],'all')<wall&&max(center(:,:,1))>wall);
reserve=rho/8*diff(angles)^2;assert(min(wall-ends(:,:,1),[],'all')<reserve);
% Verify the rotation remainder against arbitrary corner radii and angles.
rng(147);largestExcess=-Inf;
for k=1:40
 a=randn;b=a+randn*.8;corner=xy(randi(4),:)';s=linspace(0,1,101);
 R=@(t)[cos(t),-sin(t);sin(t),cos(t)];chord=R(a)*corner.*(1-s)+R(b)*corner.*s;
 arc=[cos(a+(b-a)*s)*corner(1)-sin(a+(b-a)*s)*corner(2);sin(a+(b-a)*s)*corner(1)+cos(a+(b-a)*s)*corner(2)];
 largestExcess=max(largestExcess,max(vecnorm(arc-chord))-rho*(b-a)^2/8);
end
assert(largestExcess<1e-12);
% The search seed obeys continuous heading, endpoint rest and rate bounds.
[states,controls,step,planes]=hpocp.Initialize(c,o);
assert(all(isfinite([states(:);controls(:);planes(:)]))&&step>0);
assert(max(abs(states([1 end],4:5)),[],'all')<1e-12&&max(abs(diff(states(:,3))))<pi);
assert(max(abs(controls(:,1)))<=v.amax+1e-10&&max(abs(controls(:,2)))<=v.wmax+1e-10);
assert(max(abs(hypot(planes(:,:,1),planes(:,:,2))-1),[],'all')<1e-12);
shifted=c;shifted.task.theta0=c.task.theta0+4*pi;shifted.task.thetaf=c.task.thetaf-6*pi;
[lifted,~,~,~]=hpocp.Initialize(shifted,o);
assert(abs(lifted(1,3)-shifted.task.theta0)<1e-10&&max(abs(diff(lifted(:,3))))<pi);
assert(abs(atan2(sin(lifted(end,3)-shifted.task.thetaf),cos(lifted(end,3)-shifted.task.thetaf)))<1e-8);
envelope=TestHyperplaneEnvelope();
report=struct('passed',true,'rk4_refinement_error',error,'endpoint_only_collision_rejected',true, ...
 'rotation_remainder_maximum_excess',largestExcess,'common_model_search_seed',true,'unit_interval_normals',true,'equivalent_endpoint_heading_lift',true,'continuous_corner_envelope',envelope);disp(report);
end
