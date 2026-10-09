function hit=SweptCollision(q,c)
% Appendix B between-waypoint polygon check. A convex hull conservatively
% encloses linear corner sweeps; dense pose interpolation also checks rotation.
hit=false;polygons=parking.PolygonData(c);
for k=1:size(q,1)-1
 first=parking.VehiclePolygon(q(k,:),c.vehicle);second=parking.VehiclePolygon(q(k+1,:),c.vehicle);
 points=[first(1:4,:);second(1:4,:)];indices=convhull(points(:,1),points(:,2));sweep=points(indices(1:end-1),:);
 for j=1:polygons.count
  if intersect(sweep,polygons.vertices{j}),hit=true;return;end
 end
 steps=max(1,ceil((norm(q(k+1,1:2)-q(k,1:2))+c.vehicle.length*abs(q(k+1,3)-q(k,3)))/.01));
 poses=q(k,:)+(0:steps)'/steps.*(q(k+1,:)-q(k,:));
 if ~parking.FootprintClearance(poses,c,0),hit=true;return;end
end
end
function yes=intersect(a,b)
yes=true;
for polygon={a,b}
 p=polygon{1};edges=p([2:end 1],:)-p;axes=[-edges(:,2),edges(:,1)];
 for k=1:size(axes,1)
  x=a*axes(k,:)';y=b*axes(k,:)';if min(x)>max(y)||min(y)>max(x),yes=false;return;end
 end
end
end
