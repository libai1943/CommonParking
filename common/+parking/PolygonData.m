function p=PolygonData(c)
% Separate convex polygons; never merge disconnected obstacles into one set.
M=c.obstacle.num_obs;p=struct('count',M,'vertices',{cell(M,1)},'A',{cell(M,1)},'b',{cell(M,1)},'area',zeros(M,1));
for m=1:M
 o=c.obstacle.obs{m};z=[o.x(:) o.y(:)];if norm(z(1,:)-z(end,:))<1e-10,z(end,:)=[];end
 cross=z(:,1).*z([2:end 1],2)-z([2:end 1],1).*z(:,2);
 if sum(cross)<0,z=flipud(z);end
 e=z([2:end 1],:)-z;A=[e(:,2),-e(:,1)];A=A./vecnorm(A,2,2);b=sum(A.*z,2);
 assert(all(z*A'<=b'+1e-8,'all'),'Convex decomposition required for this obstacle.');
 p.vertices{m}=z;p.A{m}=A;p.b{m}=b;p.area(m)=polyarea(z(:,1),z(:,2));
end
end
