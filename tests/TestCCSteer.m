function report=TestCCSteer()
ccp.EnsureNative();previous=rng;rng(2004064);restore=onCleanup(@()rng(previous)); %#ok<NASGU>
c=LoadCase(3);o=ccp.Config(c.vehicle);lim=[o.kappa,o.sigma];a=[randn(320,2)*5,rand(320,1)*2*pi,zeros(320,1)];b=[randn(320,2)*5,rand(320,1)*2*pi,zeros(320,1)];
endpoint=0;odeError=0;continuity=0;limitError=0;reversalError=0;
for mode={'connect','topological'}
 [curves,L]=cc_steer_mex(mode{1},a,b,lim);
 for j=1:numel(curves)
  u=curves{j};lengths=abs(u(:,1));ke=u(:,2)+u(:,3).*lengths;
  continuity=max(continuity,max([0;abs(ke(1:end-1)-u(2:end,2));abs(u(1,2));abs(ke(end))]));limitError=max([limitError;max(abs([u(:,2);ke]))-lim(1);max(abs(u(:,3)))-lim(2)]);
  q=cc_steer_mex('sample',a(j,:),u,L(j));e=q(1:3)-b(j,1:3);e(3)=atan2(sin(e(3)),cos(e(3)));endpoint=max(endpoint,max(abs(e)));
  rev=flipud(u);rev(:,2)=rev(:,2)+rev(:,3).*abs(rev(:,1));rev(:,[1 3])=-rev(:,[1 3]);back=cc_steer_mex('sample',b(j,:),rev,L(j));e=back(1:3)-a(j,1:3);e(3)=atan2(sin(e(3)),cos(e(3)));reversalError=max(reversalError,max(abs(e)));
  if j<=30
   z=a(j,1:3)';
   for k=1:size(u,1),v=u(k,:);[~,states]=ode45(@(s,q)sign(v(1))*[cos(q(3));sin(q(3));v(2)+v(3)*s],[0 abs(v(1))],z,odeset('RelTol',1e-11,'AbsTol',1e-12));z=states(end,:)';end
   e=z'-q(1:3);e(3)=atan2(sin(e(3)),cos(e(3)));odeError=max(odeError,max(abs(e)));
  end
 end
end
epsilon=10.^(-1:-1:-8)';radii=zeros(size(epsilon));
for j=1:numel(epsilon),[u,L]=cc_steer_mex('topological',[0 0 0 0],epsilon(j)*[1 1 1 0],lim);q=cc_steer_mex('sample',[0 0 0 0],u{1},linspace(0,L,1001)');radii(j)=max(vecnorm(q(:,1:2),2,2));end
assert(all(diff(radii)<0)&&radii(end)<.01&&endpoint<1e-8&&odeError<1e-7&&continuity<1e-10&&limitError<1e-10&&reversalError<1e-8);
% Independent primal QPs verify the SUPPORT-PLANE lower distance bound.
p=parking.PolygonData(c);v=c.vehicle;body=[v.lw+v.lf,v.lr,v.lb/2];maxDistanceError=0;QP=optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'ConstraintTolerance',1e-10);
for j=1:30
 pose=[-5+10*rand,-3+10*rand,-pi+2*pi*rand];box=parking.VehiclePolygon(pose,v);box=box(1:4,:);edge=box([2:4 1],:)-box;A=[edge(:,2),-edge(:,1)];A=A./vecnorm(A,2,2);bb=sum(A.*box,2);distance=inf;
 for k=1:p.count,[~,f,flag]=quadprog(2*[eye(2),-eye(2);-eye(2),eye(2)],zeros(4,1),blkdiag(A,p.A{k}),[bb;p.b{k}],[],[],[],[],[],QP);assert(flag>0);distance=min(distance,sqrt(max(0,f)));end
 lower=cc_steer_mex('clearance',pose,body,p.vertices,100);assert(lower<=distance+1e-8);maxDistanceError=max(maxDistanceError,abs(lower-distance));
end
tiny={[1.97 -.1;2.03 -.1;2.03 .1;1.97 .1]};test=o;test.checkStep=4;test.clearanceCap=10;
assert(~ccp.Edge([0 0 0 0],[4 0 0],tiny,[.1 .1 .05],test));assert(ccp.Edge([0 1 0 0],[4 0 0],tiny,[.1 .1 .05],test));
assert(maxDistanceError<1e-5);
report=struct('passed',true,'connections',640,'independent_ODE_connections',60,'endpoint_error',endpoint,'ODE_error',odeError,'all_cusp_curvature_continuity_error',continuity,'reverse_error',reversalError,'limit_violation',limitError,'QP_distance_error',maxDistanceError,'topological_epsilon',epsilon,'topological_radius',radii,'between_endpoint_collision_detected',true);disp(report);
end
