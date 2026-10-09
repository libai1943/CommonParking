function free=GeometricEdgeFree(a,b,c,options)
% Conservative continuous check of linear SE(2) interpolation. The SAT gap is
% a distance lower bound; a body-point displacement bound certifies intervals.
delta=b-a;delta(3)=atan2(sin(delta(3)),cos(delta(3)));b=a+delta;
radius=hypot(max(c.vehicle.lr,c.vehicle.lw+c.vehicle.lf),c.vehicle.lb/2);
motion=norm(delta(1:2))+radius*abs(delta(3));free=false;
if ~parking.FootprintClearance([a;b],c,options.geometricClearance),return;end
left=0;right=1;
for depth=0:options.certificateDepth
 middle=(left+right)/2;poses=a+middle.*delta;[~,gap]=parking.FootprintClearance(poses,c,0);
 if any(gap<=options.geometricClearance),return;end
 uncertain=gap<=options.geometricClearance+motion*(right-left)/2;
 if ~any(uncertain),free=true;return;end
 l=left(uncertain);r=right(uncertain);m=middle(uncertain);left=[l;m];right=[m;r];
end
end
