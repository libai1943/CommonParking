function [q,kappa,gear] = ArcPose(start,primitives,s)
% Exact evaluation of a piecewise-circular path at arbitrary absolute mileage.
ends = cumsum(abs(primitives(:,1))); starts = [0;ends(1:end-1)];
q0 = zeros(size(primitives,1),3); state = start(:)';
for j=1:size(primitives,1)
    q0(j,:) = state;
    next = parking.IntegratePrimitive(state,primitives(j,1),primitives(j,2));
    next(3) = state(3)+primitives(j,1)*primitives(j,2);
    state = next;
end
s = min(max(s(:),0),ends(end));
q = zeros(numel(s),3); kappa = zeros(size(s)); gear = kappa;
% Right-hand value at an internal primitive boundary; last point uses left.
for j=1:size(primitives,1)
    if j==size(primitives,1), mask=s>=starts(j); else, mask=s>=starts(j)&s<ends(j); end
    gear(mask) = sign(primitives(j,1)); kappa(mask) = primitives(j,2);
    q(mask,:) = parking.IntegratePrimitive(q0(j,:),gear(mask).*(s(mask)-starts(j)),primitives(j,2));
    q(mask,3) = q0(j,3)+gear(mask).*(s(mask)-starts(j))*primitives(j,2);
end
end
