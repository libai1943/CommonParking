function [q,speed,first,second,third]=Evaluate(piece,t)
t=t(:);v=piece.v;
[A,A1,A2,A3]=lamiraux.Canonical(piece.a,v*t,v);
if strcmp(piece.type,'canonical'),p=A;first=A1;second=A2;third=A3;
else
 [B,B1,B2,B3]=lamiraux.Canonical(piece.b,v*(t-1),v);
 alpha=10*t.^3-15*t.^4+6*t.^5;d1=30*t.^2.*(1-t).^2;d2=60*t-180*t.^2+120*t.^3;d3=60-360*t+360*t.^2;
 p=(1-alpha).*A+alpha.*B;
 first=(1-alpha).*A1+alpha.*B1+d1.*(B-A);
 second=(1-alpha).*A2+alpha.*B2+2*d1.*(B1-A1)+d2.*(B-A);
 third=(1-alpha).*A3+alpha.*B3+3*d1.*(B2-A2)+3*d2.*(B1-A1)+d3.*(B-A);
end
speed=hypot(first(:,1),first(:,2));direction=sign(v);
theta=atan2(direction*first(:,2),direction*first(:,1));kappa=direction*(first(:,1).*second(:,2)-first(:,2).*second(:,1))./speed.^3;
q=[p theta kappa];
end
