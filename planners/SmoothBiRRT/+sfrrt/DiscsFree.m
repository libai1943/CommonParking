function [free,gap]=DiscsFree(q,polygons,centres,radius)
x=q(:,1)+cos(q(:,3))*centres;y=q(:,2)+sin(q(:,3))*centres;
x=x(:);y=y(:);gap=inf(size(x));
for j=1:numel(polygons)
 p=polygons{j};[inside,on]=inpolygon(x,y,p(:,1),p(:,2));dist=inf(size(x));
 for k=1:size(p,1)
  a=p(k,:);delta=p(mod(k,size(p,1))+1,:)-a;t=max(0,min(1,((x-a(1))*delta(1)+(y-a(2))*delta(2))/sum(delta.^2)));
  dist=min(dist,hypot(x-a(1)-t*delta(1),y-a(2)-t*delta(2)));
 end
 dist(inside|on)=-dist(inside|on);gap=min(gap,dist-radius);
end
free=all(gap>0);
end
