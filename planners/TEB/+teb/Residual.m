function [r,parts]=Residual(z,b,env,o)
[p,dt]=teb.Unpack(z,b);v=env.vehicle;
f=[p(:,4).*cos(p(:,3)),p(:,4).*sin(p(:,3)),p(:,4).*tan(p(:,5))/v.lw];
dynamic=diff(p(:,1:3))-.5*dt.*(f(1:end-1,:)+f(2:end,:));
acceleration=diff(p(:,4))./dt;rate=diff(p(:,5))./dt;
gap=teb.Distance(p,env);middle=.5*(p(1:end-1,:)+p(2:end,:));midGap=teb.Distance(middle,env);
parts=struct('time',dt,'dynamics',dynamic,'velocity',max(0,abs(p(:,4))-v.vmax), ...
'steering',max(0,abs(p(:,5))-v.phimax),'acceleration',max(0,abs(acceleration)-v.amax), ...
'steeringRate',max(0,abs(rate)-v.wmax),'obstacles',max(0,o.clearance-gap),'midpointObstacles',max(0,o.clearance-midGap));
r=[dt;sqrt(o.dynamicsWeight)*dynamic(:);sqrt(o.limitsWeight)*[parts.velocity;parts.steering;parts.acceleration;parts.steeringRate]; ...
sqrt(o.obstacleWeight)*[parts.obstacles(:);parts.midpointObstacles(:)]];
end
