function report=ValidateHyperplaneNative(c,result)
n=result.diagnostics.native;o=result.diagnostics.options;z=n.states;u=n.controls;h=n.h;v=c.vehicle;N=size(u,1);
report=hpocp.Check(c,z,u,h,n.planes,o);assert(report.success&&N==o.intervals);
assert(abs(result.solver.objective-report.objective)<1e-7*(1+report.objective));q=result.trajectory;
difference=[q.x q.y q.theta q.v q.phi]-z;difference(:,3)=atan2(sin(difference(:,3)),cos(difference(:,3)));assert(max(abs(difference),[],'all')<1e-9);
assert(max(abs(q.t-(0:N)'*h))<1e-10&&max(abs(q.a-[u(:,1);u(end,1)]))<1e-10&&max(abs(q.omega-[u(:,2);u(end,2)]))<1e-10);
% Reconstruct stage equations, physical controls and primal supports independently.
expected=zeros(N,5);
for j=1:N
 q0=z(j,:)';k1=flow(q0,u(j,:),v.lw);k2=flow(q0+h/2*k1,u(j,:),v.lw);
 k3=flow(q0+h/2*k2,u(j,:),v.lw);k4=flow(q0+h*k3,u(j,:),v.lw);
 expected(j,:)=(q0+h/6*(k1+2*k2+2*k3+k4))';
end
report.independent_RK4_residual=max(abs(expected-z(2:end,:)),[],'all');assert(report.independent_RK4_residual<=o.feasibilityTolerance);
body=[v.lw+v.lf,v.lb/2;v.lw+v.lf,-v.lb/2;-v.lr,-v.lb/2;-v.lr,v.lb/2];
radius=max(vecnorm(body,2,2));polygons=parking.PolygonData(c);minimumSupport=Inf;maximumNormError=0;
for i=1:N
 speed=max(abs(z(i:i+1,4)));steering=max(abs(z(i:i+1,5)));curvature=tan(steering)/v.lw;
 A=abs(u(i,1))+speed^2*curvature+radius*(abs(u(i,1))*curvature+speed*abs(u(i,2))/(v.lw*cos(steering)^2)+(speed*curvature)^2);
 reserve=max(radius/8*(z(i+1,3)-z(i,3))^2,h^2/8*A)+o.supportTolerance;
 for j=1:polygons.count
  plane=reshape(n.planes(i,j,:),1,3);normal=plane(1:2);offset=plane(3);maximumNormError=max(maximumNormError,abs(norm(normal)^2-1));
  obstacleSupport=offset-max(polygons.vertices{j}*normal');
  for k=i:i+1
   angle=z(k,3);R=[cos(angle),-sin(angle);sin(angle),cos(angle)];corners=body*R'+z(k,1:2);
   minimumSupport=min([minimumSupport,obstacleSupport,min(corners*normal'-offset)-reserve]);
  end
 end
end
assert(maximumNormError<=o.feasibilityTolerance&&minimumSupport>=-o.feasibilityTolerance);
report.independent_minimum_interval_support_m=minimumSupport;report.independent_normal_squared_error=maximumNormError;
initial=result.diagnostics.initial_states;controls=result.diagnostics.initial_controls;
assert(max(abs(initial([1 end],4:5)),[],'all')<1e-10&&max(abs(diff(initial(:,3))))<pi);
assert(max(abs(controls(:,1)))<=v.amax+1e-9&&max(abs(controls(:,2)))<=v.wmax+1e-9);
% Dense linear submitted-reference footprints, distinct from control replay.
reference=zeros(0,3);
for j=1:N
 alpha=linspace(0,1,max(3,ceil(h/.001)+1))';if j<N,alpha=alpha(1:end-1);end
 reference=[reference;z(j,1:3).*(1-alpha)+z(j+1,1:3).*alpha]; %#ok<AGROW>
end
[~,referenceGap]=parking.FootprintClearance(reference,c,0);
report.linear_reference_collision_percent=100*mean(referenceGap<=0);report.linear_reference_minimum_gap_m=min(referenceGap);
assert(report.linear_reference_collision_percent==0);
[~,nodeGap]=parking.FootprintClearance(z(:,1:3),c,0);report.native_node_collision_percent=100*mean(nodeGap<=0);report.minimum_node_separating_gap_m=min(nodeGap);
globalState=z(1,:)';maximumLocalDefect=0;allReplay=zeros(0,5);options=odeset('RelTol',1e-10,'AbsTol',1e-12,'MaxStep',min(h/4,.02));
for j=1:N
 time=linspace(0,h,ceil(h/.001)+1)';start=[z(j,:)';globalState];
 [~,both]=ode45(@(~,s)[flow(s(1:5),u(j,:),v.lw);flow(s(6:10),u(j,:),v.lw)],time,start,options);
 localError=both(end,1:5)-z(j+1,:);localError(3)=atan2(sin(localError(3)),cos(localError(3)));maximumLocalDefect=max(maximumLocalDefect,max(abs(localError)));
 globalState=both(end,6:10)';if j<N,both=both(1:end-1,:);end
 allReplay=[allReplay;both(:,6:10)]; %#ok<AGROW>
end
[~,gap]=parking.FootprintClearance(allReplay(:,1:3),c,0);report.adaptive_ode_interval_endpoint_error=maximumLocalDefect;
report.adaptive_global_replay_final_error=globalState'-z(end,:);report.adaptive_global_replay_collision_percent=100*mean(gap<=0);
report.adaptive_global_replay_minimum_separating_gap_m=min(gap);report.replay_maximum_step_s=.001;
report.adaptive_global_replay_motion_violation=max([0;abs(allReplay(:,4))-v.vmax;abs(allReplay(:,5))-v.phimax]);
end
function dz=flow(z,u,lw)
dz=[z(4)*cos(z(3));z(4)*sin(z(3));z(4)*tan(z(5))/lw;u(1);u(2)];
end
