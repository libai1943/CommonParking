function free=GeometricFree(a,b,polygons,body,o)
% Continuous rigid-body certificate; angles are deliberately NOT wrapped.
delta=b-a;motion=norm(delta(1:2))+hypot(max(body(1:2)),body(3))*abs(delta(3));
% A body-aligned translation sweeps exactly one elongated rectangle.
% This removes the near-boundary subdivision cost without point sampling.
if abs(delta(3))<1e-13&&abs(delta(1)*sin(a(3))-delta(2)*cos(a(3)))<1e-10
 swept=body;swept(1:2)=swept(1:2)+norm(delta(1:2))/2;
 gap=cc_steer_mex('clearance',[(a+b)/2,0],swept,polygons,o.clearanceCap);free=gap>o.geometricClearance;return;
end
gap=cc_steer_mex('clearance',[a,0;b,0],body,polygons,o.clearanceCap);
free=false;if any(gap<=o.geometricClearance),return;end
left=0;right=1;
for depth=0:o.certificateDepth
 m=(left+right)/2;z=a+m.*delta;gap=cc_steer_mex('clearance',[z,zeros(size(z,1),1)],body,polygons,o.clearanceCap);
 if any(gap<=o.geometricClearance),return;end
 need=gap<=o.geometricClearance+motion*(right-left)/2;
 if ~any(need),free=true;return;end
 l=left(need);r=right(need);m=m(need);left=[l;m];right=[m;r];
end
end
