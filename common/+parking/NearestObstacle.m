function [distance,gradient,label]=NearestObstacle(P,c)
% Exact point-to-polygon signed distance and its piecewise analytical gradient.
n=size(P,1);distance=inf(n,1);gradient=zeros(n,2);label=zeros(n,1);
for j=1:c.obstacle.num_obs
    o=c.obstacle.obs{j};A=[o.x(1:end-1)' o.y(1:end-1)'];B=A([2:end 1],:);E=B-A;
    closest=zeros(n,2);best=inf(n,1);
    for k=1:size(A,1)
        u=max(0,min(1,((P-A(k,:))*E(k,:)')/sum(E(k,:).^2)));
        Q=A(k,:)+u.*E(k,:);dd=sum((P-Q).^2,2);take=dd<best;
        best(take)=dd(take);closest(take,:)=Q(take,:);
    end
    d=sqrt(best);inside=inpolygon(P(:,1),P(:,2),o.x,o.y);
    signDistance=ones(n,1);signDistance(inside)=-1;
    g=signDistance.*(P-closest)./max(d,1e-12);d=d.*signDistance;
    take=d<distance;distance(take)=d(take);gradient(take,:)=g(take,:);label(take)=j;
end
end
