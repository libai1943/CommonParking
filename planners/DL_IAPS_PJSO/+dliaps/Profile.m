function p=Profile(P,gear)
% Standard centred/one-sided finite differences of the discrete positions.
n=size(P,1);p.s=[0;cumsum(vecnorm(diff(P),2,2))];
assert(all(diff(p.s)>1e-9),'CommonParking:CollapsedPath','Smoothing collapsed a path interval.');
left=[1;(1:n-2)';n-1];right=[2;(3:n)';n];
first=(P(right,:)-P(left,:))./(p.s(right)-p.s(left));
second=(first(right,:)-first(left,:))./(p.s(right)-p.s(left));
p.x=P(:,1);p.y=P(:,2);p.theta=unwrap(atan2(gear*first(:,2),gear*first(:,1)));
p.kappa=gear*(first(:,1).*second(:,2)-first(:,2).*second(:,1))./sum(first.^2,2).^(3/2);
p.gear=gear;
end
