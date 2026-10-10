function [z,u,h,planes]=Initialize(c,o)
% Common-model search seed; geometric separating normals have unit norm.
raw=parking.SearchHybridAStar(c,o.search);assert(raw.success,'Hybrid A* initialization failed.');
q=hpocp.Seed(c,raw,o.intervals+1);h=q.t(end)/o.intervals;z=[q.x q.y q.theta q.v q.phi];
u=[diff(q.v)/h,diff(q.phi)/h];polygons=parking.PolygonData(c);planes=zeros(o.intervals,polygons.count,3);
frames=hpocp.Frames(z(:,1:3),c.vehicle);
for i=1:size(z,1)-1
 B=[reshape(frames(i:i+1,:,1)',[],1),reshape(frames(i:i+1,:,2)',[],1)];
 for j=1:polygons.count
  A=polygons.vertices{j};edge=[diff([A;A(1,:)]);diff([B;B(1,:)])];edge=edge(vecnorm(edge,2,2)>1e-12,:);axes=[-edge(:,2),edge(:,1)]./vecnorm(edge,2,2);axes=[axes;-axes];
  pa=A*axes';pb=B*axes';gaps=min(pb,[],1)-max(pa,[],1);[~,k]=max(gaps);normal=axes(k,:);mu=.5*(min(pb(:,k))+max(pa(:,k)));
  planes(i,j,:)=[normal,mu];
 end
end
end
