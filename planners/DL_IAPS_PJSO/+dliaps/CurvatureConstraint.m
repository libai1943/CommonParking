function [g,J]=CurvatureConstraint(P,kappaMax,scale)
% Entire quartic Eq. (2f), not a frozen reference-distance approximation.
n=size(P,1);v=P(2:end-1,:)-P(1:end-2,:);a=2*P(2:end-1,:)-P(1:end-2,:)-P(3:end,:);
d2=sum(v.^2,2);g=(sum(a.^2,2)-kappaMax^2*d2.^2)/scale;
if nargout<2,return;end
left=(-2*a+4*kappaMax^2*d2.*v)/scale;
middle=(4*a-4*kappaMax^2*d2.*v)/scale;right=-2*a/scale;
r=(1:n-2)';J=sparse([r;r;r;r;r;r],[r;r+1;r+2;r+n;r+n+1;r+n+2], ...
 [left(:,1);middle(:,1);right(:,1);left(:,2);middle(:,2);right(:,2)],n-2,2*n);
end
