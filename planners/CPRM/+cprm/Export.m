function path=Export(pieces,vehicle,spacing)
directions=[pieces.direction];ends=[0 cumsum([pieces.length])];cuts=[1 find(diff(directions)~=0)+1 numel(pieces)+1];s=0;gear=zeros(0,1);cusps=1;
for j=1:numel(cuts)-1
 left=ends(cuts(j));right=ends(cuts(j+1));count=max(1,ceil((right-left)/spacing));nodes=linspace(left,right,count+1)';
 s=[s;nodes(2:end)];gear=[gear;repmat(directions(cuts(j)),count,1)];cusps(end+1,1)=numel(s); %#ok<AGROW>
end
q=cprm.PathPose(pieces,s);path=struct('s',s,'x',q(:,1),'y',q(:,2),'theta',unwrap(q(:,3)), ...
 'phi',atan(vehicle.lw*q(:,4)),'gear',gear,'cusp_indices',cusps);
end
