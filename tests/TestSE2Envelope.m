function report=TestSE2Envelope()
% A rotating corner's chord-interpolation remainder is <= |r|*dtheta^2/8.
c=LoadCase(1);v=c.vehicle;corners=[v.lw+v.lf,v.lb/2;v.lw+v.lf,-v.lb/2;-v.lr,-v.lb/2;-v.lr,v.lb/2];rho=max(vecnorm(corners,2,2));
headings=linspace(-5*pi,5*pi,17);turns=[0,logspace(-7,-1,7),.5,1,pi-1e-5];fractions=linspace(0,1,101)';worst=0;
for heading=headings
 for turn=[-turns,turns]
  R=@(q)[cos(q),-sin(q);sin(q),cos(q)];start=corners*R(heading)';finish=corners*R(heading+turn)';
  for k=1:4
   theta=heading+fractions*turn;actual=[corners(k,1)*cos(theta)-corners(k,2)*sin(theta),corners(k,1)*sin(theta)+corners(k,2)*cos(theta)];
   chord=(1-fractions).*start(k,:)+fractions.*finish(k,:);excess=max(vecnorm(actual-chord,2,2))-rho*turn^2/8;worst=max(worst,excess);
  end
 end
end
assert(worst<1e-12);report=struct('passed',true,'largest_bound_violation_m',worst,'corner_radius_m',rho,'angle_lifts_and_signed_turns_checked',true);disp(report);
end
