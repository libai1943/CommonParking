function d=Metric(q,target,kappa)
% Equation (1): circle orientations are axes, with difference in [0,pi/2].
angle=q(:,3)-target(:,3);angle=abs(atan2(sin(angle),cos(angle)));angle=min(angle,pi-angle);
d=max(vecnorm(q(:,1:2)-target(:,1:2),2,2),angle/kappa);
end
