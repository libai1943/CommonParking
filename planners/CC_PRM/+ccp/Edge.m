function free=Edge(q,u,polygons,body,o)
% Complete-body distance plus arc-length Lipschitz certificate on each interval.
% Every body point has speed <= 1 + rear-axle-to-corner radius * max curvature.
free=false;L=sum(abs(u(:,1)));if L<1e-12,free=true;return;end
ends=[0;cumsum(abs(u(:,1)))];s=unique([linspace(0,L,ceil(L/o.checkStep)+1)';ends]);
poses=cc_steer_mex('sample',q,u,s);gap=cc_steer_mex('clearance',poses,body,polygons,o.clearanceCap);
if any(gap<=o.margin+1e-10),return;end
rate=1+hypot(max(body(1:2)),body(3))*o.kappa;
while true
 unresolved=find(min(gap(1:end-1),gap(2:end))-rate*diff(s)/2<=o.margin+1e-10);
 if isempty(unresolved),free=true;return;end
 if any(s(unresolved+1)-s(unresolved)<o.minimumStep),return;end
 mid=(s(unresolved)+s(unresolved+1))/2;poses=cc_steer_mex('sample',q,u,mid);g=cc_steer_mex('clearance',poses,body,polygons,o.clearanceCap);if any(g<=o.margin+1e-10),return;end
 [s,index]=sort([s;mid]);gap=[gap;g];gap=gap(index);
end
end
