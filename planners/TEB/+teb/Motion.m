function m=Motion(p,dt,o)
d=diff(p(:,1:2));m.chord=vecnorm(d,2,2);m.angle=atan2(sin(diff(p(:,3))),cos(diff(p(:,3))));
projection=sum(d.*[cos(p(1:end-1,3)),sin(p(1:end-1,3))],2);
z=o.sigmoidScale*projection;m.gamma=z./(1+abs(z));m.v=m.chord.*m.gamma./dt;m.yaw=m.angle./dt;
m.a=[m.v(1)/dt(1);2*diff(m.v)./(dt(1:end-1)+dt(2:end));-m.v(end)/dt(end)];
m.yawAcceleration=[m.yaw(1)/dt(1);2*diff(m.yaw)./(dt(1:end-1)+dt(2:end));-m.yaw(end)/dt(end)];
m.nonholonomic=(cos(p(1:end-1,3))+cos(p(2:end,3))).*d(:,2)- ...
 (sin(p(1:end-1,3))+sin(p(2:end,3))).*d(:,1);
end
