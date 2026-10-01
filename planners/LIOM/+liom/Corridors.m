function [boxes,info] = Corridors(c,reference,discs,options)
% Algorithms 1-2; bounded eight-ray seed relocation follows supplied source.
n=numel(reference.x);d=discs.count;o=discs.offsets;
ct=cos(reference.theta);st=sin(reference.theta);
x=reference.x+ct*o(:,1)'-st*o(:,2)';y=reference.y+st*o(:,1)'+ct*o(:,2)';
points=[x(:) y(:)];original=points;polygons=parking.PolygonData(c);
threshold=discs.radius+options.clearance;
valid=cp.BoxObstacleDistance([points(:,1) points(:,1) points(:,2) points(:,2)],polygons)>=threshold;
relocated=~valid;boxes=[];
for ring=1:ceil(options.nudgeLimit/options.nudgeStep)
    if all(valid),break;end
    for direction=1:8
        ids=find(~valid);if isempty(ids),break;end
        angle=mod(direction,8)*pi/4;
        q=original(ids,:)+ring*options.nudgeStep*[cos(angle) sin(angle)];
        safe=cp.BoxObstacleDistance([q(:,1) q(:,1) q(:,2) q(:,2)],polygons)>=threshold;
        points(ids(safe),:)=q(safe,:);valid(ids(safe))=true;
    end
end
info=struct('success',all(valid),'relocated_seed_count',sum(relocated), ...
    'maximum_relocation',max(vecnorm(points-original,2,2)),'disc_count',d);
if ~info.success,return;end
flat=[points(:,1) points(:,1) points(:,2) points(:,2)];active=true(n*d,4);lengths=zeros(n*d,4);
for sweep=1:ceil(options.corridorExtent/options.corridorStep)
    if ~any(active,'all'),break;end
    for side=[4 1 3 2]
        ids=find(active(:,side));if isempty(ids),continue;end
        trial=flat(ids,:);delta=options.corridorStep;if ismember(side,[1 3]),delta=-delta;end
        trial(:,side)=trial(:,side)+delta;
        safe=cp.BoxObstacleDistance(trial,polygons)>=threshold;
        active(ids(~safe),side)=false;accepted=ids(safe);flat(accepted,:)=trial(safe,:);
        lengths(accepted,side)=lengths(accepted,side)+abs(delta);
        active(lengths(:,side)>=options.corridorExtent-1e-10,side)=false;
    end
end
boxes=reshape(flat,[n d 4]);
end
