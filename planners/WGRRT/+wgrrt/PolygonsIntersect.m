function hit=PolygonsIntersect(a,b)
% Separating-axis test for two convex polygons; contact counts as collision.
hit=true;
for polygon={a,b}
 p=polygon{1};edge=p([2:end 1],:)-p;axis=[-edge(:,2),edge(:,1)];
 for j=1:size(axis,1)
  x=a*axis(j,:)';y=b*axis(j,:)';
  if min(x)>max(y)+1e-10||min(y)>max(x)+1e-10,hit=false;return;end
 end
end
end

