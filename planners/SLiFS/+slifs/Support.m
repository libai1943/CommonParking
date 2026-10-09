function [normal,offset,distance]=Support(points,polygon)
% Closest-point supporting halfspaces of a convex polygon, outward normals.
% Each row satisfies normal*polygon_vertex <= offset.
if sum(polygon(:,1).*circshift(polygon(:,2),-1)-polygon(:,2).*circshift(polygon(:,1),-1))<0,polygon=flipud(polygon);end
N=size(points,1);distance=inf(N,1);nearest=zeros(N,2);edgeNormal=zeros(N,2);inside=true(N,1);
for e=1:size(polygon,1)
 a=polygon(e,:);b=polygon(mod(e,size(polygon,1))+1,:);d=b-a;n=[d(2),-d(1)]/norm(d);
 fraction=max(0,min(1,(points-a)*d'/dot(d,d)));q=a+fraction.*d;dist=sqrt(sum((points-q).^2,2));
 better=dist<distance;distance(better)=dist(better);nearest(better,:)=q(better,:);edgeNormal(better,:)=repmat(n,nnz(better),1);
 inside=inside&((points-a)*n'<=1e-12);
end
normal=(points-nearest)./max(distance,1e-12);useEdge=inside|distance<1e-12;normal(useEdge,:)=edgeNormal(useEdge,:);
offset=sum(normal.*nearest,2);distance(inside)=-distance(inside);
end
