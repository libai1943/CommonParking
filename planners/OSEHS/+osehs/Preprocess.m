function route=Preprocess(route,kappa)
% Section IV: preferred travel signs and places requiring extra maneuvering.
Q=route.q;n=size(Q,1);direction=zeros(n,1);distance=zeros(n-1,1);maneuver=false(n,1);
for j=1:n-1
 delta=Q(j+1,1:2)-Q(j,1:2);direction(j)=sign(delta*[cos(Q(j,3));sin(Q(j,3))]);
 distance(j)=osehs.Metric(Q(j,:),Q(j+1,:),kappa);
 angle=abs(atan2(sin(Q(j+1,3)-Q(j,3)),cos(Q(j+1,3)-Q(j,3))));angle=min(angle,pi-angle);
 if angle>kappa*norm(delta)+1e-10,maneuver(j:j+1)=true;end
end
direction(n)=direction(n-1);direction(maneuver)=0;route.direction=direction;route.remaining=[flipud(cumsum(flipud(distance)));0];
end
