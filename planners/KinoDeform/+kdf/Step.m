function [next,J,K]=Step(q,u,h,sgn,L)
% Exact derivative of this RK4 step. q = [x,y,theta,v,phi].
if nargout==1
 a=field(q,u);b=field(q+h.*a/2,u);c=field(q+h.*b/2,u);d=field(q+h.*c,u);
 next=q+h.*(a+2*b+2*c+d)/6;return;
end
assert(size(q,1)==1);D=[eye(5),zeros(5,2)];B=sgn*[zeros(3,2);eye(2)];
[a,A]=field(q,u);Da=A*D+[zeros(5),B];
[b,A]=field(q+h*a/2,u);Db=A*(D+h*Da/2)+[zeros(5),B];
[c,A]=field(q+h*b/2,u);Dc=A*(D+h*Db/2)+[zeros(5),B];
[d,A]=field(q+h*c,u);Dd=A*(D+h*Dc)+[zeros(5),B];
next=q+h*(a+2*b+2*c+d)/6;D=D+h*(Da+2*Db+2*Dc+Dd)/6;J=D(:,1:5);K=D(:,6:7);
 function [f,A]=field(z,control)
  th=z(:,3);vel=z(:,4);phi=z(:,5);f=sgn*[vel.*cos(th),vel.*sin(th),vel.*tan(phi)/L,control];
  if nargout>1,A=zeros(5);A(1,3)=-vel*sin(th);A(1,4)=cos(th);A(2,3)=vel*cos(th);A(2,4)=sin(th);A(3,4)=tan(phi)/L;A(3,5)=vel/(L*cos(phi)^2);A=sgn*A;end
 end
end
