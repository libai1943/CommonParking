function report=TestDLIAPS()
SetupCommonParking();o=dliaps.Config();common=BenchmarkConfig();v=common.vehicle;
old=rng;cleanup=onCleanup(@()rng(old));rng(184); %#ok<NASGU>
maximumDerivativeError=0;
for trial=1:20
 P=cumsum(.1*randn(20,2));[~,J]=dliaps.CurvatureConstraint(P,v.kappa_max,.1^4);
 delta=randn(size(P));h=1e-7;
 fd=(dliaps.CurvatureConstraint(P+h*delta,v.kappa_max,.1^4)-dliaps.CurvatureConstraint(P-h*delta,v.kappa_max,.1^4))/(2*h);
 maximumDerivativeError=max(maximumDerivativeError,norm(fd-J*delta(:),Inf)/max(1,norm(fd,Inf)));
end
assert(maximumDerivativeError<1e-7);
c=LoadCase(1);c.obstacle.num_obs=0;c.obstacle.obs={};
theta=.73;s=linspace(0,5,51)';P=[-7+cos(theta)*s,3+sin(theta)*s];
[profile,inner]=dliaps.Smooth([P,repmat(theta,51,1)],1,c,o,tic);assert(inner.success);
assert(max(abs([profile.x-P(:,1);profile.y-P(:,2);profile.theta-theta]))<1e-6);
for fraction=[.7 .9]
 angle=linspace(0,pi/3,41)';radius=1/(fraction*v.kappa_max);
 reference=[radius*sin(angle),radius*(1-cos(angle)),angle];
 [p,details]=dliaps.Smooth(reference,1,c,o,tic);assert(details.success);
 assert(details.iterations{end}.maximum_normalized_quartic_residual<=o.constraintTolerance);
 assert(max(abs([p.x([1 end])-reference([1 end],1);p.y([1 end])-reference([1 end],2);p.theta([1 end])-reference([1 end],3)]))<1e-7);
end
maxSpeedResidual=0;maxIntegrationError=0;
for gear=[-1,1]
 for L=[.1,1,5,20]
  p=dliaps.Profile([linspace(0,L,11)',zeros(11,1)],gear);
  [speed,info]=dliaps.Speed(p,v,o);assert(info.success);q=dliaps.Trajectory(p,speed,v,o);
  assert(all(diff(q.t)>1e-10)&&all(diff(q.t+1000)>1e-10));
  maxSpeedResidual=max(maxSpeedResidual,info.maximum_residual);
  assert(max(abs([q.x(1);q.x(end)-L;q.v([1 end]);q.a([1 end])]))<1e-7);
  for i=1:numel(speed.jerk)
   h=speed.t(i+1)-speed.t(i);
   integration=integral(@(tau)speed.v(i)+speed.a(i)*tau+.5*speed.jerk(i)*tau.^2,0,h,'AbsTol',1e-12);
   maxIntegrationError=max(maxIntegrationError,abs(integration-speed.s(i+1)+speed.s(i)));
  end
 end
end
assert(maxIntegrationError<1e-7);
report=struct('passed',true,'quartic_gradient_relative_error',maximumDerivativeError, ...
 'maximum_speed_qp_residual',maxSpeedResidual,'speed_integration_error',maxIntegrationError);
disp(report);
end
