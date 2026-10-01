function [boxes,info] = DiscCorridors(c,reference,discs,options)
% ECC 2020 box expansion: up, left, down, right, fixed increment and limit.
n=numel(reference.x);d=discs.count;o=discs.offsets;
ct=cos(reference.theta);st=sin(reference.theta);
x=reference.x+ct*o(:,1)'-st*o(:,2)';
y=reference.y+st*o(:,1)'+ct*o(:,2)';
flat=[x(:) x(:) y(:) y(:)];polygons=parking.PolygonData(c);
gap=cp.BoxObstacleDistance(flat,polygons)-discs.radius-options.clearance;
info=struct('success',false,'minimum_seed_disc_clearance',min(gap), ...
    'disc_count',d,'partition',discs.partition,'radius',discs.radius,'message','');
boxes=[];
if any(gap<0)
    info.message='This covering-disc partition is too conservative for the reference path.';return;
end
active=true(n*d,4);lengths=zeros(n*d,4);
for sweep=1:ceil(options.corridorExtent/options.corridorStep)
    if ~any(active,'all'),break;end
    for side=[4 1 3 2]
        ids=find(active(:,side));if isempty(ids),continue;end
        trial=flat(ids,:);delta=options.corridorStep;
        if ismember(side,[1 3]),delta=-delta;end
        trial(:,side)=trial(:,side)+delta;
        safe=cp.BoxObstacleDistance(trial,polygons)>=discs.radius+options.clearance;
        active(ids(~safe),side)=false;
        accepted=ids(safe);flat(accepted,:)=trial(safe,:);
        lengths(accepted,side)=lengths(accepted,side)+abs(delta);
        active(lengths(:,side)>=options.corridorExtent-1e-10,side)=false;
    end
end
boxes=reshape(flat,[n d 4]);info.success=true;info.message='Reference-centred fixed safe corridors constructed.';
end
