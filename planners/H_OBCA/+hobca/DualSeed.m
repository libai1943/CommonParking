function [lambda,mu,clearance] = DualSeed(c,poses,polygons)
% Exact geometric solution of the disjoint-polytopes distance dual.
% The subsequent auxiliary NLP independently refines/checks this warm start.
N=size(poses,1);M=polygons.count;nv=cellfun(@(p)size(p,1),polygons.vertices);
lambda=zeros(N,sum(nv));mu=zeros(N,M,4);clearance=zeros(N,M);start=[0;cumsum(nv)];
for i=1:N
    q=poses(i,:);body=parking.VehiclePolygon(q,c.vehicle);body=body(1:4,:);
    R=[cos(q(3)) -sin(q(3));sin(q(3)) cos(q(3))];
    for j=1:M
        obstacle=polygons.vertices{j};A=polygons.A{j};b=polygons.b{j};
        best=inf;carPoint=[];obsPoint=[];
        for k=1:4
            [value,p]=nearest(body(k,:),obstacle);
            if value<best,best=value;carPoint=body(k,:);obsPoint=p;end
        end
        for k=1:size(obstacle,1)
            [value,p]=nearest(obstacle(k,:),body);
            if value<best,best=value;carPoint=p;obsPoint=obstacle(k,:);end
        end
        if best<1e-10,error('CommonParking:OverlappingDualSeed','Dual initializer requires a separated Hybrid A* seed.');end
        normal=(carPoint-obsPoint)'/best;
        active=find(abs(A*obsPoint'-b)<1e-7);
        weights=lsqnonneg(A(active,:)',normal);l=zeros(size(b));l(active)=weights;
        local=-R'*normal;m=[max(local(1),0);max(-local(1),0);max(local(2),0);max(-local(2),0)];
        g=[c.vehicle.lw+c.vehicle.lf;c.vehicle.lr;c.vehicle.lb/2;c.vehicle.lb/2];
        certificate=-g'*m+(A*q(1:2)'-b)'*l;
        if norm(A'*l-normal)>1e-6||certificate<0
            error('CommonParking:InvalidDualSeed','Seed polygon dual certificate is inconsistent.');
        end
        lambda(i,start(j)+(1:nv(j)))=l;mu(i,j,:)=m;clearance(i,j)=certificate;
    end
end
end
function [distance,point]=nearest(q,polygon)
edges=polygon([2:end 1],:)-polygon;
t=max(0,min(1,sum((q-polygon).*edges,2)./sum(edges.^2,2)));
p=polygon+t.*edges;[distance,index]=min(vecnorm(p-q,2,2));point=p(index,:);
end
