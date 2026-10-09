function [valid,info]=Certify(piece,c,o,collision)
% Continuous bounds, with conservative rejection when subdivision is exhausted.
valid=false;left=piece.range(1);right=piece.range(2);count=0;highest=0;
bodyRadius=hypot(max(c.vehicle.lr,c.vehicle.lw+c.vehicle.lf),c.vehicle.lb/2);cap=c.vehicle.kappa_max;
for depth=0:o.curveDepth
 middle=(left+right)/2;[q,speed]=lamiraux.Evaluate(piece,middle);count=count+numel(middle);
 if any(~isfinite(q),'all')||any(speed<=o.minimumParameterSpeed)||any(abs(q(:,4))>=cap),break;end
 [speedMin,speedMax,kappaMax]=lamiraux.DerivativeBounds(piece,left,right);
 uncertain=speedMin<=o.minimumParameterSpeed|kappaMax>cap*(1-1e-10);
 if collision
  [~,gap]=parking.FootprintClearance(q(:,1:3),c,0);if any(gap<=o.curveClearance),break;end
  movement=(right-left)/2.*speedMax*(1+bodyRadius*cap);uncertain=uncertain|gap<=o.curveClearance+movement;
 end
 accepted=~uncertain;if any(accepted),highest=max(highest,max(kappaMax(accepted)));end
 if ~any(uncertain),valid=true;break;end
 if depth==o.curveDepth,break;end
 l=left(uncertain);r=right(uncertain);m=middle(uncertain);left=[l;m];right=[m;r];
end
info=struct('passed',valid,'intervals_checked',count,'maximum_depth',depth,'curvature_upper_bound',highest);
end
