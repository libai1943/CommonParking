function [ok,gap] = FootprintClearance(q,c,margin)
% Separating-axis test for the full rectangular ego body and convex obstacles.
% gap is a conservative lower bound on Euclidean separation (not exact distance).
if nargin<3, margin=0; end
v=c.vehicle; ct=cos(q(:,3)); st=sin(q(:,3));
cx=q(:,1)+(v.length/2-v.lr)*ct;
cy=q(:,2)+(v.length/2-v.lr)*st;
gap=inf(size(q,1),1);
for j=1:c.obstacle.num_obs
    ob=c.obstacle.obs{j}; p=[ob.x(1:end-1)' ob.y(1:end-1)'];
    edge=p([2:end 1],:)-p;
    axes=[-edge(:,2) edge(:,1)]; axes=axes./vecnorm(axes,2,2);
    best=-inf(size(q,1),1);
    for k=1:size(axes,1)
        ax=axes(k,1); ay=axes(k,2); proj=p*axes(k,:)';
        mid=cx*ax+cy*ay;
        rad=v.length/2*abs(ct*ax+st*ay)+v.lb/2*abs(-st*ax+ct*ay);
        best=max(best,max(min(proj)-mid-rad,mid-rad-max(proj)));
    end
    for k=1:2
        if k==1, ax=ct; ay=st; rad=v.length/2; else, ax=-st; ay=ct; rad=v.lb/2; end
        proj=ax*p(:,1)'+ay*p(:,2)'; mid=cx.*ax+cy.*ay;
        best=max(best,max(min(proj,[],2)-mid-rad,mid-rad-max(proj,[],2)));
    end
    gap=min(gap,best);
end
ok=all(gap>margin);
end
