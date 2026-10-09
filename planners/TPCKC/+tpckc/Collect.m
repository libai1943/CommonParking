function keys=Collect(q,c,propagate)
% Both V2P directions, including boundary contact. No edge-edge SAT repair.
polys=parking.PolygonData(c);v=c.vehicle;n=numel(q.x);keys=zeros(0,4);
body=[v.lw+v.lf,v.lb/2;v.lw+v.lf,-v.lb/2;-v.lr,-v.lb/2;-v.lr,v.lb/2];
co=cos(q.theta);si=sin(q.theta);
for i=1:polys.count
 A=polys.A{i};b=polys.b{i};p=polys.vertices{i};
 for j=1:4
  xy=[q.x+co*body(j,1)-si*body(j,2),q.y+si*body(j,1)+co*body(j,2)];inside=all(xy*A'<=b',2);k=find(inside);k=k(k>1&k<n);
  keys=[keys; k,ones(numel(k),1),repmat([i j],numel(k),1)]; %#ok<AGROW>
 end
 for j=1:size(p,1)
  dx=p(j,1)-q.x;dy=p(j,2)-q.y;xx=co.*dx+si.*dy;yy=-si.*dx+co.*dy;
  k=find(xx<=v.lw+v.lf&xx>=-v.lr&abs(yy)<=v.lb/2);k=k(k>1&k<n);
  keys=[keys;k,2*ones(numel(k),1),repmat([i j],numel(k),1)]; %#ok<AGROW>
 end
end
if propagate>0&&~isempty(keys)
 propagatedRows=cell(2*propagate+1,1);for offset=-propagate:propagate,z=keys;z(:,1)=z(:,1)+offset;propagatedRows{offset+propagate+1}=z(z(:,1)>1&z(:,1)<n,:);end
 keys=vertcat(propagatedRows{:});
end
keys=unique(keys,'rows');
end
