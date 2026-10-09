function path=ExportPath(native,vehicle,origin,R,heading)
% Uniform true mileage by gear run, preserving zero-length-phase removal.
counts=accumarray(native.phase,1,[numel(native.lengths) 1]);h=native.lengths(native.phase)./counts(native.phase);
active=h>1e-10;indices=find(active);step=h(active);gears=native.gear(native.phase(active));
assert(~isempty(indices),'The optimized path has zero length.');
ends=[0;cumsum(step)];cuts=[1;find(diff(gears)~=0)+1;numel(gears)+1];
s=0;gear=zeros(0,1);cusps=1;cfg=BenchmarkConfig();
for j=1:numel(cuts)-1
    left=ends(cuts(j));right=ends(cuts(j+1));number=max(1,ceil((right-left)/cfg.output.path_spacing_max_m));
    nodes=linspace(left,right,number+1)';s=[s;nodes(2:end)];gear=[gear;repmat(gears(cuts(j)),number,1)];cusps(end+1,1)=numel(s); %#ok<AGROW>
end
interval=discretize(s,[-inf;ends(2:end-1);inf]);interval(s==ends(end))=numel(indices);original=indices(interval);
distance=s-ends(interval);z=lattice.Step(native.states(original,:),native.controls(original),distance,gears(interval),vehicle,1);
% The OCP nodes are authoritative at the exact endpoints and cusps.
for cusp=cusps'
    boundary=find(abs(ends-s(cusp))<1e-9,1);
    if boundary==numel(ends),z(cusp,:)=native.states(indices(end)+1,:);else,z(cusp,:)=native.states(indices(boundary),:);end
end
xy=z(:,1:2)*R'+origin;
path=struct('s',s,'x',xy(:,1),'y',xy(:,2),'theta',z(:,3)+heading,'phi',z(:,4),'gear',gear,'cusp_indices',cusps);
end
