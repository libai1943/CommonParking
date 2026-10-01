function value = BarycentricInterpolate(nodes,weights,data,query)
% Stable evaluation of the unique global interpolation polynomial.
nodes=nodes(:);weights=weights(:);query=query(:);value=zeros(numel(query),size(data,2));
for first=1:512:numel(query)
    indices=first:min(first+511,numel(query));delta=query(indices)-nodes';
    [distance,nearest]=min(abs(delta),[],2);hit=distance<8*eps;
    delta(hit,:)=1;basis=weights'./delta;
    value(indices,:)=(basis*data)./sum(basis,2);
    value(indices(hit),:)=data(nearest(hit),:);
end
end
