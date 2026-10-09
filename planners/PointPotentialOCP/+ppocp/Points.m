function points=Points(c,target)
polys=parking.PolygonData(c);total=0;
for k=1:polys.count,q=polys.vertices{k};total=total+sum(vecnorm(q([2:end 1],:)-q,2,2));end
points=zeros(0,2);
for k=1:polys.count
 q=polys.vertices{k};for j=1:size(q,1),a=q(j,:);b=q(mod(j,size(q,1))+1,:);n=max(1,ceil(target*norm(b-a)/total));s=(0:n-1)'/n;points=[points;a+s.*(b-a)];end %#ok<AGROW>
end
end
