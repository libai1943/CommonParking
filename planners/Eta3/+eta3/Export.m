function p=Export(pieces,vehicle,spacing)
s=[];q=[];gear=[];cusps=1;offset=0;
for piece=pieces
 distance=linspace(0,piece.length,max(1,ceil(piece.length/spacing))+1)';u=eta3.Parameter(piece,distance);
 pose=eta3.Evaluate(piece,u);pose(:,3)=unwrap(pose(:,3));
 if ~isempty(q),pose(:,3)=pose(:,3)+2*pi*round((q(end,3)-pose(1,3))/(2*pi));s=s(1:end-1);q=q(1:end-1,:);end
 s=[s;offset+distance];q=[q;pose];gear=[gear;piece.gear*ones(numel(distance)-1,1)]; %#ok<AGROW>
 offset=offset+piece.length;cusps(end+1,1)=numel(s); %#ok<AGROW>
end
p=struct('s',s,'x',q(:,1),'y',q(:,2),'theta',q(:,3),'phi',atan(vehicle.lw*q(:,4)),'gear',gear,'cusp_indices',cusps);
end
