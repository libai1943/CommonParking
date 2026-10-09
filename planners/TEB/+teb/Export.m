function [q,info]=Export(b,env,o)
% Whole-band output adapter; pose/time knots are exactly the optimizer's.
m=teb.Motion(b.pose,b.dt,o);t=[0;cumsum(b.dt)];
velocity=[0;.5*(m.v(1:end-1)+m.v(2:end));0];yaw=[0;.5*(m.yaw(1:end-1)+m.yaw(2:end));0];
phi=zeros(size(t));active=abs(velocity)>1e-10;phi(active)=atan(env.vehicle.lw*yaw(active)./velocity(active));
if any(~active(2:end-1)&abs(yaw(2:end-1))>1e-8)
 q=[];info=struct('success',false,'code','stationary_yaw_not_bicycle');return;
end
for index=[1,numel(t)]
 if index==1,j=1;else,j=numel(b.dt);end
 if abs(m.v(j))>1e-10,phi(index)=atan(env.vehicle.lw*m.yaw(j)/m.v(j));end
end
q=struct('t',t,'x',b.pose(:,1),'y',b.pose(:,2),'theta',unwrap(b.pose(:,3)), ...
 'v',velocity,'phi',phi,'a',gradient(velocity,t),'omega',gradient(phi,t));
info=struct('success',true,'code','whole_band_export','interval_motion',m);
end
