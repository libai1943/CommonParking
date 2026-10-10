function [speed,info]=Speed(p,v,o)
% Equations (4)-(6), with exact terminal rest as in the authors' source.
L=p.s(end);h=o.timeStep;amax=min(v.a_accel,v.a_brake);
if p.gear>0,vmax=v.v_forward;else,vmax=v.v_reverse;end
vmax=min(vmax,sqrt(o.maxLateralAcceleration/max(max(abs(p.kappa)),realmin)));
steeringSlope=max(abs(diff(atan(v.lw*p.kappa))./diff(p.s)));
n=max(4,floor(o.horizonFactor*(vmax^2+L*amax)/(amax*vmax*h)));
D=spdiags([-ones(n-1,1),ones(n-1,1)],0:1,n-1,n)/h;
H=2*blkdiag(o.distanceWeight*speye(n),sparse(n,n),o.accelerationWeight*speye(n)+o.jerkWeight*(D'*D));
f=[-2*o.distanceWeight*L*ones(n,1);zeros(2*n,1)];
r=(1:n-1)';Aeq=sparse([r;r;r;r;r;r+n-1;r+n-1;r+n-1;r+n-1], ...
 [r+1;r;r+n;r+2*n;r+2*n+1;r+n+1;r+n;r+2*n;r+2*n+1], ...
 [ones(n-1,1);-ones(n-1,1);-h*ones(n-1,1);-h*h/3*ones(n-1,1);-h*h/6*ones(n-1,1);ones(n-1,1);-ones(n-1,1);-h/2*ones(n-1,1);-h/2*ones(n-1,1)],2*(n-1),3*n);
lower=[zeros(2*n,1);-amax*ones(n,1)];upper=[L*ones(n,1);vmax*ones(n,1);amax*ones(n,1)];
fixed=[1,n,n+1,2*n,2*n+1,3*n];values=[0,L,0,0,0,0];lower(fixed)=values;upper(fixed)=values;
% The middle Bernstein control bounds the complete quadratic velocity,
% including extrema between the speed QP knots.
middle=sparse([r;r],[n+r;2*n+r],[ones(n-1,1);h/2*ones(n-1,1)],n-1,3*n);
A=[sparse(n-1,2*n),D;sparse(n-1,2*n),-D;middle;-middle];
b=[o.maxJerk*ones(2*(n-1),1);vmax*ones(n-1,1);zeros(n-1,1)];
[z,objective,flag,output]=quadprog(H,f,A,b,Aeq,zeros(2*(n-1),1),lower,upper,[],o.qp);
speed=[];info=struct('success',false,'exitflag',flag,'iterations',output.iterations,'nodes',n,'step_s',h,'speed_cap',vmax,'maximum_steering_slope',steeringSlope,'objective',objective);
if flag<=0||isempty(z),return;end
residual=max([0;abs(Aeq*z);A*z-b;lower-z;z-upper]);info.maximum_residual=residual;
info.success=residual<1e-6;
speed=struct('t',(0:n-1)'*h,'s',z(1:n),'v',z(n+1:2*n),'a',z(2*n+1:end),'jerk',diff(z(2*n+1:end))/h);
% Preserve the native PJSO solution and uniformly dilate its clock only
% when the common steering-rate limit requires it. All cubic dynamics,
% endpoint rest and speed/acceleration/jerk bounds are preserved.
peak=dliaps.SteeringPeak(p,speed,v.lw);scale=max(1,peak/v.wmax*(1+1e-8));
info.native_speed=speed;info.native_speed_cap=vmax;info.native_step_s=h;
info.clock_scale=scale;info.maximum_rate_before_scaling=peak;
speed.t=speed.t*scale;speed.v=speed.v/scale;speed.a=speed.a/scale^2;speed.jerk=speed.jerk/scale^3;
info.step_s=h*scale;info.speed_cap=vmax/scale;
end
