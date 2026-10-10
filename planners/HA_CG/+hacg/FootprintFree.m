function ok=FootprintFree(q,g,margin)
% Same separating axes and strict margin as parking.FootprintClearance.
% Only a Boolean is needed; stop as soon as any pose is blocked.
v=g.vehicle;ct=cos(q(:,3));st=sin(q(:,3));cx=q(:,1)+(v.length/2-v.lr)*ct;cy=q(:,2)+(v.length/2-v.lr)*st;
for j=1:numel(g.polygons)
 ob=g.polygons{j};p=ob.p;best=-inf(size(q,1),1);
 for k=1:size(ob.axes,1)
  ax=ob.axes(k,1);ay=ob.axes(k,2);mid=cx*ax+cy*ay;
  rad=v.length/2*abs(ct*ax+st*ay)+v.lb/2*abs(-st*ax+ct*ay);
  best=max(best,max(ob.lo(k)-mid-rad,mid-rad-ob.hi(k)));
 end
 for k=1:2
  if k==1,ax=ct;ay=st;rad=v.length/2;else,ax=-st;ay=ct;rad=v.lb/2;end
  projection=ax*p(:,1)'+ay*p(:,2)';mid=cx.*ax+cy.*ay;
  best=max(best,max(min(projection,[],2)-mid-rad,mid-rad-max(projection,[],2)));
 end
 if ~all(best>margin),ok=false;return;end
end
ok=true;
end
