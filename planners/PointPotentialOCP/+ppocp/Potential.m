function [value,gradient]=Potential(q,points,v)
% Printed ICRA2001 Eq.(4): choose x branch for the larger x clearance.
% Coordinates are reflected into their quadrant; side lengths are positive.
n=size(q,2);value=zeros(1,n);gradient=zeros(3,n);ct=cos(q(3,:));st=sin(q(3,:));
for k=1:size(points,1)
 dx=points(k,1)-q(1,:);dy=points(k,2)-q(2,:);x=ct.*dx+st.*dy;y=-st.*dx+ct.*dy;
 rx=repmat(v.lw+v.lf,1,n);rx(x<0)=v.lr;ry=v.lb/2;ax=abs(x);ay=abs(y);inside=ax<rx&ay<ry;
 bx=inside&(rx-ax>=ry-ay);by=inside&~bx;gx=zeros(1,n);gy=gx;
 value(bx)=value(bx)+1-ax(bx)./rx(bx);value(by)=value(by)+1-ay(by)/ry;
 gx(bx)=-sign(x(bx))./rx(bx);gy(by)=-sign(y(by))/ry;
 gradient=gradient+[-ct.*gx+st.*gy;-st.*gx-ct.*gy;y.*gx-x.*gy];
end
end
