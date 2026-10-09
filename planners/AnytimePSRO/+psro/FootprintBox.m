function box=FootprintBox(pose,positionRadius,angleRadius,body)
% Exact axis extrema of the rotating corners plus the translation box.
lower=pose(3)-angleRadius;upper=pose(3)+angleRadius;points=zeros(0,2);
for j=1:size(body,1)
 offset=atan2(body(j,2),body(j,1));angles=[lower upper];
 indices=ceil((lower+offset)/(pi/2)):floor((upper+offset)/(pi/2));angles=[angles indices*pi/2-offset]; %#ok<AGROW>
 radius=hypot(body(j,1),body(j,2));points=[points;radius*[cos(angles(:)+offset),sin(angles(:)+offset)]]; %#ok<AGROW>
end
box=[pose(1:2)+min(points,[],1)-positionRadius,pose(1:2)+max(points,[],1)+positionRadius];
end
