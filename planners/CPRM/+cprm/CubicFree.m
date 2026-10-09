function valid=CubicFree(piece,c,o)
valid=false;if piece.minimum_speed<=o.minimumSpeed||piece.maximum_curvature>c.vehicle.kappa_max,return;end
bodyRadius=hypot(max(c.vehicle.lr,c.vehicle.lw+c.vehicle.lf),c.vehicle.lb/2);left=0;right=1;coef=piece.coefficients;
for depth=0:o.collisionDepth
 middle=(left+right)/2;q=cprm.Evaluate(piece,middle);[~,gap]=parking.FootprintClearance(q(:,1:3),c,0);if any(gap<=o.collisionClearance),return;end
 high=zeros(numel(left),2);
 for axis=1:2
  a=3*coef(4,axis);b=2*coef(3,axis);d=coef(2,axis);values=[a*left.^2+b*left+d,a*right.^2+b*right+d];
  high(:,axis)=max(abs(values),[],2);if abs(a)>1e-14,critical=-b/(2*a);inside=left<critical&right>critical;high(inside,axis)=max(high(inside,axis),abs(a*critical^2+b*critical+d));end
 end
 speedBound=hypot(high(:,1),high(:,2))+1e-10;move=speedBound*(1+bodyRadius*piece.maximum_curvature).*(right-left)/2;
 uncertain=gap<=o.collisionClearance+move;if ~any(uncertain),valid=true;return;end
 l=left(uncertain);r=right(uncertain);m=middle(uncertain);left=[l;m];right=[m;r];
end
end
