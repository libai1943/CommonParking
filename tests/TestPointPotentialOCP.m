function report=TestPointPotentialOCP()
SetupCommonParking();c=LoadCase(3);v=c.vehicle;o=ppocp.Config();rng(2001164);
q=randn(5,9);q(4:5,:)=q(4:5,:)/5;[~,A]=ppocp.Model(q,v.lw);modelError=0;
for j=1:5,p=zeros(size(q));p(j,:)=1i*1e-20;f=ppocp.Model(q+p,v.lw);modelError=max(modelError,max(abs(imag(f)/1e-20-squeeze(A(:,j,:))),[],'all'));end
points=[.5,.3;-.5,-.3;1.8,-.7;-.4,.75;4,3];poses=[zeros(2,4);0,.2,-.4,.7];[potential,g]=ppocp.Potential(poses,points,v);assert(all(potential>0));potentialError=0;
for j=1:3,d=zeros(size(poses));d(j,:)=1e-6;fd=(ppocp.Potential(poses+d,points,v)-ppocp.Potential(poses-d,points,v))/2e-6;potentialError=max(potentialError,max(abs(fd-g(j,:))));end
angle=.83;R=[cos(angle),-sin(angle);sin(angle),cos(angle)];origin=[-4;7];rotated=[R*poses(1:2,:)+origin;poses(3,:)+angle];p2=points*R'+origin';assert(max(abs(ppocp.Potential(rotated,p2,v)-potential))<1e-12);
assert(ppocp.Potential([0;0;0],[v.lw+v.lf,0],v)==0);assert(ppocp.Potential([0;0;0],[.5,.3],v)>0);
% The literal printed branch selects the larger longitudinal clearance.
assert(abs(ppocp.Potential([0;0;0],[.5,.3],v)-(1-.5/(v.lw+v.lf)))<1e-14);
ctx=struct('grid',linspace(0,1,9),'points',points,'vehicle',v);z=[q(:);20];[~,~,GI,GE]=ppocp.Constraints(z,ctx);d=randn(size(z));[ap,bp]=ppocp.Constraints(z+1e-6*d,ctx);[am,bm]=ppocp.Constraints(z-1e-6*d,ctx);constraintError=max(norm((ap-am)/2e-6-GI'*d,inf),norm((bp-bm)/2e-6-GE'*d,inf));
assert(modelError<1e-12&&potentialError<1e-6&&constraintError<1e-5);
N=81;t=linspace(0,9,N);speed=zeros(1,N);phi=.3*min(t,1);s=zeros(1,N);a=t>1&t<=5;b=t>5;speed(a)=.5*(t(a)-1);s(a)=.25*(t(a)-1).^2;speed(b)=2-.5*(t(b)-5);s(b)=4+2*(t(b)-5)-.25*(t(b)-5).^2;kappa=tan(.3)/v.lw;
states=[sin(kappa*s)/kappa;(1-cos(kappa*s))/kappa;kappa*s;speed;phi];c.task.x0=0;c.task.y0=0;c.task.theta0=0;c.task.xf=states(1,end);c.task.yf=states(2,end);c.task.thetaf=states(3,end);angles=linspace(.25,.65,10)';radius=1/kappa;points=[(radius-2)*sin(angles),radius-(radius-2)*cos(angles)];
[z,info]=ppocp.Solve(c,N,points,[states(:);9],o);assert(info.success&&z(end)<9);states=reshape(z(1:end-1),5,N);assert(max(abs(states(4:5,[1 end])),[],'all')<1e-10);[dense,~]=ppocp.Dense(z,v,linspace(0,z(end),N));interpolationError=max(abs(dense-states),[],'all');assert(interpolationError<1e-12);
testTimes=(linspace(0,z(end),N-1)+z(end)/(N-1)*.31);testTimes=testTimes(testTimes<z(end));[~,d]=ppocp.Dense(z,v,testTimes);fd=(ppocp.Dense(z,v,testTimes+1e-6)-ppocp.Dense(z,v,testTimes-1e-6))/2e-6;denseError=max(abs(fd-d),[],'all');assert(denseError<1e-7);
frame=struct('R',eye(2),'origin',[0 0],'angle',0);states=zeros(5,4);states(1,:)=[0 .1 .2 .3];states(4,:)=[.2 -.7 .4 0];[p,n]=ppocp.Output([states(:);3],v,frame,o);assert(numel(n.cusp_indices)==2&&all(p.v(n.cusp_indices)==0));assert(max(abs(p.t(n.cusp_indices)-[.2/.9;1+.7/1.1]))<1e-14&&all(diff(p.t)>0));
states(4,:)=[0 .2 -1e-15 .4];[p,n]=ppocp.Output([states(:);3],v,frame,o);assert(p.t(1)==0&&p.t(end)==3&&all(diff(p.t)>0)&&all(p.v(n.cusp_indices)==0));
report=struct('model_jacobian_error',modelError,'point_potential_gradient_error',potentialError,'collocation_jacobian_error',constraintError,'curved_fixture_sqp_success',info.success,'curved_fixture_time_s',z(end),'curved_constraint_residual',info.infeasibility,'native_node_interpolation_error',interpolationError,'dense_derivative_error',denseError,'exact_off_grid_cusps',true,'coincident_timestamp_merge',true);disp(report);
end
