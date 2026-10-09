function gap=DiscGap(q,polygons,centres,radius,bounds)
% Signed center-to-polygon distance minus radius, for every disk and polygon.
x=q(:,1)+cos(q(:,3))*centres;y=q(:,2)+sin(q(:,3))*centres;x=x(:);y=y(:);
gap=zeros(numel(x),numel(polygons)+4);
for j=1:numel(polygons)
 p=polygons{j};[inside,on]=inpolygon(x,y,p(:,1),p(:,2));distance=inf(size(x));
 for k=1:size(p,1)
  a=p(k,:);delta=p(mod(k,size(p,1))+1,:)-a;
  t=max(0,min(1,((x-a(1))*delta(1)+(y-a(2))*delta(2))/sum(delta.^2)));
  distance=min(distance,hypot(x-a(1)-t*delta(1),y-a(2)-t*delta(2)));
 end
 distance(inside|on)=-distance(inside|on);gap(:,j)=distance-radius;
end
gap(:,end-3:end)=[x-radius-bounds(1,1),bounds(2,1)-x-radius,y-radius-bounds(1,2),bounds(2,2)-y-radius];
end
