function [body,obstacles]=Circles(c)
% Paper's three-circle car cover. Each obstacle rectangle is also covered.
discs=cp.CoveringDiscs(c.vehicle,3,1);body=[discs.offsets,repmat(discs.radius,3,1)];
polygons=parking.PolygonData(c);obstacles=zeros(0,3);
for m=1:polygons.count
    p=polygons.vertices{m};axis=p(2,:)-p(1,:);axis=axis/norm(axis);R=[axis',[-axis(2);axis(1)]];
    local=p*R;lower=min(local);upper=max(local);width=upper-lower;
    if width(1)>=width(2),parts=[ceil(width(1)/width(2)) 1];else,parts=[1 ceil(width(2)/width(1))];end
    cellWidth=width./parts;radius=norm(cellWidth)/2;
    [ix,iy]=ndgrid(1:parts(1),1:parts(2));centres=lower+([ix(:) iy(:)]-.5).*cellWidth;
    centres=centres*R';obstacles=[obstacles;centres,repmat(radius,size(centres,1),1)]; %#ok<AGROW>
end
end
