function [p,first,second,third]=Canonical(q,s,v)
% Canonical curve gamma(q,s), differentiated w.r.t. a parameter with s'=v.
s=s(:);angle=q(3)+q(4)*s;e=[cos(angle) sin(angle)];normal=[-e(:,2) e(:,1)];
if abs(q(4))<1e-12,p=q(1:2)+s.*[cos(q(3)) sin(q(3))];
else,p=q(1:2)+[(sin(angle)-sin(q(3)))/q(4),-(cos(angle)-cos(q(3)))/q(4)];end
first=v*e;second=v^2*q(4)*normal;third=-v^3*q(4)^2*e;
end
