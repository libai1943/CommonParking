function distance=BoundaryDistance(point,g)
% Search nodes have already passed full-body clearance, so the reference
% point is outside every obstacle. Unsigned boundary distance equals signed
% polygon distance here. This helper is not an inside/outside classifier.
u=max(0,min(1,sum((point-g.starts).*g.edges,2)./g.squares));
q=g.starts+u.*g.edges;distance=sqrt(min(sum((point-q).^2,2)));
end
