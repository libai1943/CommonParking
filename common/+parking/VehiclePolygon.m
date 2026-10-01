function p = VehiclePolygon(q,v)
% Four unique counterclockwise vertices, then repeat the first to close.
local = [-v.lr -v.lb/2; v.lw+v.lf -v.lb/2; ...
    v.lw+v.lf v.lb/2; -v.lr v.lb/2];
R = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))];
p = local*R' + q(1:2);
p(5,:) = p(1,:);
end
