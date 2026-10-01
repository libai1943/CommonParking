function path=Path(start,controls,vehicle,maxSpacing)
% Exact clothoid/circle/line geometry, uniform mileage in each gear run.
controls=controls(abs(controls(:,1))>1e-12,:);direction=sign(controls(:,1));
ends=[0;cumsum(abs(controls(:,1)))];cuts=[1;find(diff(direction)~=0)+1;size(controls,1)+1];
s=0;gear=zeros(0,1);cusps=1;
for j=1:numel(cuts)-1
    left=ends(cuts(j));right=ends(cuts(j+1));count=max(1,ceil((right-left)/maxSpacing));
    samples=linspace(left,right,count+1)';s=[s;samples(2:end)]; %#ok<AGROW>
    gear=[gear;repmat(direction(cuts(j)),count,1)];cusps(end+1,1)=numel(s); %#ok<AGROW>
end
q=hc_steer_mex('sample',start,controls,s);
path=struct('s',s,'x',q(:,1),'y',q(:,2),'theta',q(:,3),'phi',atan(vehicle.lw*q(:,4)), ...
    'gear',gear,'cusp_indices',cusps);
end
