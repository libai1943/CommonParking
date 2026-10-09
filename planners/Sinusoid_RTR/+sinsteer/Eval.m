function [q,s]=Eval(p,t)
% Pose/steering and unsigned physical mileage at a sinusoid parameter value.
t=t(:);
if p.n==0
 x=p.xy(1)+p.a*t;y=p.xy(2)+p.increments(1)*t;alpha=repmat(p.alpha,size(t));phi=zeros(size(t));s=p.length*t;
else
 sector=min(3,floor(t/(pi/2)));left=sector*pi/2;h=t-left;[z,w]=sinsteer.Gauss(32);u=left+h.*z';[dy,ds]=sinsteer.Integrand(p,u);
 cumulative=[zeros(1,2);cumsum(p.increments)];y=p.xy(2)+cumulative(sector+1,1)+h.*(dy*w);s=p.scale*(cumulative(sector+1,2)+h.*(ds*w));
 x=p.xy(1)+p.a*(1-cos(t));alpha=p.alpha+p.a*sinsteer.H(t,p.b,p.n);phi=p.b*sin(p.n*t);
end
R=[cos(p.rotation),-sin(p.rotation);sin(p.rotation),cos(p.rotation)];xy=p.scale*[x,y]*R'+p.origin;q=[xy,asin(alpha)+p.rotation,phi];
end
