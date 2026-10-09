function [pieces,success]=Steer(a,b,c,o)
% Section III: Steer* blends canonical curves; Steer adds one canonical tail.
pieces=struct([]);success=false;
if max(abs([a(1:2)-b(1:2),atan2(sin(a(3)-b(3)),cos(a(3)-b(3))),a(4)-b(4)]))<1e-11,success=true;return;end
delta=b(1:2)-a(1:2);heading=abs(atan2(sin(b(3)-a(3)),cos(b(3)-a(3))));lateral=abs(delta*[-sin(a(3));cos(a(3))]);
scale=max([.002,norm(delta),sqrt(lateral/c.vehicle.kappa_max),heading/c.vehicle.kappa_max]);
offsets=[0,reshape([1;-1]*(scale*o.cuspScales),1,[])];offsets=offsets(abs(offsets)<=o.cuspOffsetMax|offsets==0);
for offset=offsets
 targetPose=parking.IntegratePrimitive(b(1:3),offset,b(4));target=[targetPose,b(4)];
 if abs(a(4))<1e-12,projection=(target(1:2)-a(1:2))*[cos(a(3));sin(a(3))];
 else
  centre=a(1:2)+[-sin(a(3)),cos(a(3))]/a(4);u=a(1:2)-centre;v=target(1:2)-centre;
  projection=atan2(u(1)*v(2)-u(2)*v(1),dot(u,v))/a(4);
 end
 if abs(projection)<o.minimumParameterSpeed,continue;end
 blend=lamiraux.Segment(a,target,projection,'blend');
 if ~lamiraux.Certify(blend,c,o,false),continue;end
 candidate=blend;
 if offset~=0,candidate(2)=lamiraux.Segment(target,b,-offset,'canonical');end
 % The local steering curve is selected by kinematic admissibility. If it
 % collides, the outer holonomic approximation subdivides the failed interval.
 for j=1:numel(candidate)
  if ~lamiraux.Certify(candidate(j),c,o,true),return;end
 end
 for j=1:numel(candidate),candidate(j)=lamiraux.Prepare(candidate(j));end
 pieces=candidate;success=true;return;
end
end
