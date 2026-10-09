function [lambda,mu,d,info] = DualSeed(c,poses,polygons,o)
% Exact solution of Eq. (5), followed by Proposition III.1 normalization.
% Given Euclidean distance D, the optimal normal magnitude is beta*D/2:
% min_{r>=0} r^2/beta-r*D. The polygon support dual gives its direction.
[lambda,mu,D] = hobca.DualSeed(c,poses,polygons);
counts=cellfun(@(x)size(x,1),polygons.A);last=cumsum(counts);first=last-counts+1;
magnitude=o.dualBeta*D/2;
assert(all(D>0,'all'),'Temporal initializer overlaps an obstacle.');
scale=magnitude./max(1,magnitude);
for j=1:polygons.count
    lambda(:,first(j):last(j))=lambda(:,first(j):last(j)).*scale(:,j);
    mu(:,j,:)=mu(:,j,:).*scale(:,j);
end
d=-D.*scale;
info=struct('distance',D,'unnormalized_normal_magnitude',magnitude, ...
    'unconstrained_QP_objective',-o.dualBeta*D.^2/4,'normalized_scale',scale);
end
