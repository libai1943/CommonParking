function [free,index]=Edge(phases,polygons,body,o)
index=sinsteer.Index(phases);free=false;L=index.length;if L<1e-12,free=true;return;end
s=unique([linspace(0,L,ceil(L/o.checkStep)+1)';index.segments(:,4);L]);q=sinsteer.Sample(phases,index,s);gap=cc_steer_mex('clearance',q,body,polygons,o.clearanceCap);
if any(gap<=o.margin+1e-10),return;end
rate=1+hypot(max(body(1:2)),body(3))*o.kappa;
while true
 need=find(min(gap(1:end-1),gap(2:end))-rate*diff(s)/2<=o.margin+1e-10);if isempty(need),free=true;return;end
 if any(s(need+1)-s(need)<o.minimumStep),return;end
 mid=(s(need)+s(need+1))/2;q=sinsteer.Sample(phases,index,mid);g=cc_steer_mex('clearance',q,body,polygons,o.clearanceCap);if any(g<=o.margin+1e-10),return;end
 [s,ix]=sort([s;mid]);gap=[gap;g];gap=gap(ix);
end
end
