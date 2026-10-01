function CheckCase(c)
% Finite CCW polygons; no redundant vertices or overlapping obstacle interiors.
% Physical walls/curbs can meet exactly at their boundaries.
assert(c.obstacle.num_obs==numel(c.obstacle.obs));v=c.vehicle;
assert(strcmp(v.reference_point,'rear_axle_midpoint'));
for j=1:c.obstacle.num_obs
    o=c.obstacle.obs{j};p=[o.x(:) o.y(:)];
    assert(all(isfinite(p),'all')&&max(abs(p),[],'all')<30);
    assert(size(p,1)==5&&o.num_grids==4&&norm(p(1,:)-p(end,:))<1e-10);
    edge=diff(p);len=vecnorm(edge,2,2);
    assert(min(len)>0.3&&abs(dot(edge(1,:),edge(2,:)))<1e-8);
    signedArea=sum(p(1:4,1).*p(2:5,2)-p(2:5,1).*p(1:4,2))/2;
    assert(signedArea>0&&abs(o.area-signedArea)<1e-7);
    if strcmp(o.kind,'parked_car')
        assert(all(abs(sort(len)-sort([v.length;v.length;v.lb;v.lb]))<1e-8));
    end
    for k=1:j-1
        b=c.obstacle.obs{k};r=[b.x(1:4)' b.y(1:4)'];
        axes=[-edge(:,2) edge(:,1)];e=diff([r;r(1,:)]);axes=[axes;-e(:,2) e(:,1)];
        separate=false;
        for a=1:size(axes,1)
            u=p(1:4,:)*axes(a,:)';w=r*axes(a,:)';
            if max(u)<=min(w)+1e-8||max(w)<=min(u)+1e-8,separate=true;break;end
        end
        assert(separate,'Obstacles %d and %d overlap.',j,k);
    end
end
t=c.task;q=[t.x0 t.y0 t.theta0;t.xf t.yf t.thetaf];
[ok,gap]=parking.FootprintClearance(q,c,0.08);
assert(all(isfinite(q),'all')&&ok,'Invalid endpoints, separation %.4f / %.4f.',gap(1),gap(2));
end
