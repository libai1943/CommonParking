function edges=EdgeMatrix(library)
edges=zeros(numel(library.primitives),5);
for j=1:size(edges,1)
    p=library.primitives{j};edges(j,:)=[p.from p.to p.delta/library.options.grid p.cost];
end
assert(max(abs(edges(:,1:4)-round(edges(:,1:4))),[],'all')<1e-8);
edges(:,1:4)=round(edges(:,1:4));
end
