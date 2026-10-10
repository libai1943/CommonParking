function report=TestHyperplaneEnvelope()
SetupCommonParking();c=LoadCase(1);v=c.vehicle;body=[v.lw+v.lf,v.lb/2;v.lw+v.lf,-v.lb/2;-v.lr,-v.lb/2;-v.lr,v.lb/2];rho=max(vecnorm(body,2,2));rng(14732);maximumExcess=-Inf;
for k=1:30
 h=.05+.5*rand;z0=[randn(3,1);1.5*(2*rand-1);.4*(2*rand-1)];u=[.4*(2*rand-1);.3*(2*rand-1)];
 t=linspace(0,h,101)';[~,z]=ode45(@(~,q)[q(4)*cos(q(3));q(4)*sin(q(3));q(4)*tan(q(5))/v.lw;u],t,z0,odeset('RelTol',1e-12,'AbsTol',1e-13));
 speed=max(abs(z([1 end],4)));steering=max(abs(z([1 end],5)));curvature=tan(steering)/v.lw;
 bound=abs(u(1))+speed^2*curvature+rho*(abs(u(1))*curvature+speed*abs(u(2))/(v.lw*cos(steering)^2)+(speed*curvature)^2);
 for j=1:4
  p=z(:,1:2)+[cos(z(:,3))*body(j,1)-sin(z(:,3))*body(j,2),sin(z(:,3))*body(j,1)+cos(z(:,3))*body(j,2)];
  line=p(1,:).*(1-t/h)+p(end,:).*t/h;maximumExcess=max(maximumExcess,max(vecnorm(p-line,2,2))-h^2/8*bound);
 end
end
assert(maximumExcess<=1e-10);report=struct('passed',true,'maximum_corner_chord_error_excess_m',maximumExcess);disp(report);
end
