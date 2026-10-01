function distance = BoxObstacleDistance(boxes,polygons)
% Exact Euclidean distance from axis-aligned boxes to convex polygons.
% Degenerate boxes (points or line segments) are supported.
n=size(boxes,1);distance=inf(n,1);
cx=(boxes(:,1)+boxes(:,2))/2;cy=(boxes(:,3)+boxes(:,4))/2;
rx=(boxes(:,2)-boxes(:,1))/2;ry=(boxes(:,4)-boxes(:,3))/2;
for j=1:polygons.count
    p=polygons.vertices{j};lo=min(p);hi=max(p);
    broad=hypot(max([lo(1)-boxes(:,2),boxes(:,1)-hi(1),zeros(n,1)],[],2), ...
        max([lo(2)-boxes(:,4),boxes(:,3)-hi(2),zeros(n,1)],[],2));
    ids=find(broad<distance);if isempty(ids),continue;end
    b=boxes(ids,:);x=cx(ids);y=cy(ids);hx=rx(ids);hy=ry(ids);
    separated=b(:,2)<lo(1)|b(:,1)>hi(1)|b(:,4)<lo(2)|b(:,3)>hi(2);
    edges=p([2:end 1],:)-p;
    for k=1:size(p,1)
        normal=[-edges(k,2),edges(k,1)];projections=p*normal';
        middle=x*normal(1)+y*normal(2);radius=hx*abs(normal(1))+hy*abs(normal(2));
        separated=separated|middle+radius<min(projections)|middle-radius>max(projections);
    end
    value=inf(numel(ids),1);value(~separated)=0;
    active=find(separated);
    if ~isempty(active)
        ba=b(active,:);count=numel(active);best=inf(count,1);
        % Polygon vertices to an axis-aligned rectangle.
        for k=1:size(p,1)
            dx=max([ba(:,1)-p(k,1),p(k,1)-ba(:,2),zeros(count,1)],[],2);
            dy=max([ba(:,3)-p(k,2),p(k,2)-ba(:,4),zeros(count,1)],[],2);
            best=min(best,hypot(dx,dy));
        end
        % Rectangle vertices to polygon segments.
        for corner=[1 3;2 3;2 4;1 4]'
            q=ba(:,corner');
            for k=1:size(p,1)
                edge=edges(k,:);u=max(0,min(1,((q-p(k,:))*edge')/dot(edge,edge)));
                best=min(best,vecnorm(q-p(k,:)-u.*edge,2,2));
            end
        end
        value(active)=best;
    end
    distance(ids)=min(distance(ids),value);
end
end
