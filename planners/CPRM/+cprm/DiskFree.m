function valid=DiskFree(points,c,radius)
% The paper explicitly permits a small orientation-free disk in control space.
valid=true(size(points,1),1);polygons=parking.PolygonData(c);
for j=1:polygons.count
 p=polygons.vertices{j};distance=inf(size(points,1),1);
 for edge=1:size(p,1)
  a=p(edge,:);d=p(mod(edge,size(p,1))+1,:)-a;t=max(0,min(1,(points-a)*d'/sum(d.^2)));foot=a+t.*d;
  distance=min(distance,hypot(points(:,1)-foot(:,1),points(:,2)-foot(:,2)));
 end
 valid=valid&~inpolygon(points(:,1),points(:,2),p(:,1),p(:,2))&distance>radius;
end
end
