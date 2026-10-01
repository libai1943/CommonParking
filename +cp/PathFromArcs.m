function p = PathFromArcs(start,primitives,vehicle,maxSpacing)
% Uniform mileage in EACH gear run, exact cusps and exact task endpoints.
primitives = primitives(abs(primitives(:,1))>1e-10,:);
d = sign(primitives(:,1));
arcEnds = [0;cumsum(abs(primitives(:,1)))];
cuts = [1;find(diff(d)~=0)+1;numel(d)+1];
s = 0; gear = zeros(0,1); cusps = 1;
for j=1:numel(cuts)-1
    left = arcEnds(cuts(j)); right = arcEnds(cuts(j+1));
    count = max(1,ceil((right-left)/maxSpacing));
    nodes = linspace(left,right,count+1)';
    s = [s;nodes(2:end)]; %#ok<AGROW>
    gear = [gear;repmat(d(cuts(j)),count,1)]; %#ok<AGROW>
    cusps(end+1,1) = numel(s); %#ok<AGROW>
end
[q,kappa] = cp.ArcPose(start,primitives,s);
p = struct('s',s,'x',q(:,1),'y',q(:,2),'theta',q(:,3), ...
    'phi',atan(vehicle.lw*kappa),'gear',gear,'cusp_indices',cusps, ...
    'geometry',struct('type','piecewise_circular','start',start, ...
                     'primitives',primitives));
end
