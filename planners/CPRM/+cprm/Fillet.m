function [primitives,start,goal]=Fillet(a,b,p,direction)
% Circular tangent fillet through the two edge midpoints around shared CP p.
u=a-p;v=b-p;la=norm(u);lb=norm(v);u=u/la;v=v/lb;beta=acos(max(-1,min(1,dot(u,v))));
start=[a,atan2(-u(2),-u(1))+(direction<0)*pi];goal=[b,atan2(v(2),v(1))+(direction<0)*pi];
if beta<1e-8,primitives=zeros(0,2);return;end
if pi-beta<1e-8,primitives=[direction*(la+lb),0];return;end
radius=min(la,lb)*tan(beta/2);turn=sign(-u(1)*v(2)+u(2)*v(1));
primitives=[direction*max(0,la-lb),0;direction*radius*(pi-beta),direction*turn/radius;direction*max(0,lb-la),0];
primitives=primitives(abs(primitives(:,1))>1e-10,:);
end
