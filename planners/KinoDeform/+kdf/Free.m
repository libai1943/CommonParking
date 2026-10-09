function ok=Free(Q,t,u,sgn,body,polygons,v,o)
% Full rectangular body; constant input on all supplied intervals.
ok=false;
if any(abs(Q(:,4))>v.vmax+1e-10)||any(abs(Q(:,5))>v.phimax+1e-10),return;end
gap=cc_steer_mex('clearance',Q(:,1:3),body,polygons,1);
if any(gap<=o.margin),return;end
left=Q(1:end-1,:);right=Q(2:end,:);dt=diff(t);gleft=gap(1:end-1);gright=gap(2:end);R=hypot(max(body(1:2)),body(3));
while ~isempty(dt)
 rate=max(abs([left(:,4),right(:,4)]),[],2).*(1+R*tan(max(abs([left(:,5),right(:,5)]),[],2))/v.lw);
 bad=min(gleft,gright)<=rate.*dt/2+o.margin;
 if ~any(bad),ok=true;return;end
 left=left(bad,:);right=right(bad,:);dt=dt(bad);gleft=gleft(bad);gright=gright(bad);
 if any(dt<o.minimumTime),return;end
 mid=kdf.Step(left,repmat(u,size(left,1),1),dt/2,sgn,v.lw);gm=cc_steer_mex('clearance',mid(:,1:3),body,polygons,1);
 if any(gm<=o.margin),return;end
 left=[left;mid];right=[mid;right];gleft=[gleft;gm];gright=[gm;gright];dt=[dt;dt]/2;
end
ok=true;
end
