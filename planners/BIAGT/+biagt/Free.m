function ok=Free(q,arcs,c,o,body,polygons)
% Adaptive full-body swept-arc certificate, using support-plane lower gaps.
ok=false;R=hypot(max(body(1:2)),body(3));
for k=1:size(arcs,1)
 L=abs(arcs(k,1));d=sign(arcs(k,1));curvature=arcs(k,2);if L<1e-12,continue;end
 s=linspace(0,L,max(1,ceil(L/o.sampleStep))+1)';Q=biagt.Advance(q,d*s,curvature);gap=cc_steer_mex('clearance',Q,body,polygons,1);
 if any(gap<=o.margin),return;end
 left=s(1:end-1);right=s(2:end);gl=gap(1:end-1);gr=gap(2:end);rate=1+R*abs(curvature);
 while ~isempty(left)
  bad=min(gl,gr)<=rate*(right-left)/2+o.margin;
  if ~any(bad),break;end
  left=left(bad);right=right(bad);gl=gl(bad);gr=gr(bad);if any(right-left<o.minimumStep),return;end
  mid=(left+right)/2;gm=cc_steer_mex('clearance',biagt.Advance(q,d*mid,curvature),body,polygons,1);if any(gm<=o.margin),return;end
  left=[left;mid];right=[mid;right];gl=[gl;gm];gr=[gm;gr];
 end
 q=Q(end,:);
end
ok=true;
end
