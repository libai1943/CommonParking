function cost=FootprintCost(q,grid,body)
% Maximum raster value in cells intersecting the complete vehicle rectangle.
% SAT includes each cell's half-width, so narrow corner overlaps count.
cost=zeros(size(q,1),1);h=grid.resolution/2;p=grid.active_centres;
offset=(body(1)-body(2))/2;a=(body(1)+body(2))/2;b=body(3);
for j=1:size(q,1)
 ct=cos(q(j,3));st=sin(q(j,3));centre=q(j,1:2)+offset*[ct st];d=p-centre;
 mask=abs(d(:,1))<=a*abs(ct)+b*abs(st)+h & abs(d(:,2))<=a*abs(st)+b*abs(ct)+h;
 ids=find(mask);z=d(mask,:);pad=h*(abs(ct)+abs(st));
 hit=abs(z(:,1)*ct+z(:,2)*st)<=a+pad & abs(-z(:,1)*st+z(:,2)*ct)<=b+pad;
 if any(hit),cost(j)=max(grid.active_cost(ids(hit)));end
end
end
