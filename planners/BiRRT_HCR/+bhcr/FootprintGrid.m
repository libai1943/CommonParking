function grid=FootprintGrid(c,o)
% Raster obstacle-distance penalty. The article does not specify its kernel.
p=parking.PolygonData(c);xy=vertcat(p.vertices{:});h=o.mapResolution;
lower=floor((min(xy)-o.inflationRadius-h)/h)*h;
upper=ceil((max(xy)+o.inflationRadius+h)/h)*h;
x=lower(1):h:upper(1);y=lower(2):h:upper(2);[X,Y]=ndgrid(x,y);
distance=inf(size(X));inside=false(size(X));
for j=1:numel(p.vertices)
 v=p.vertices{j};inside=inside|inpolygon(X,Y,v(:,1),v(:,2));
 for edge=1:size(v,1)
  a=v(edge,:);b=v(mod(edge,size(v,1))+1,:);d=b-a;
  t=max(0,min(1,((X-a(1))*d(1)+(Y-a(2))*d(2))/sum(d.^2)));
  distance=min(distance,hypot(X-a(1)-t*d(1),Y-a(2)-t*d(2)));
 end
end
distance(inside)=0;cost=max(0,1-distance/o.inflationRadius);
active=cost>0;grid=struct('resolution',h,'inflation_radius',o.inflationRadius,...
 'origin',lower,'size',size(cost),'active_centres',[X(active),Y(active)],'active_cost',cost(active));
end
