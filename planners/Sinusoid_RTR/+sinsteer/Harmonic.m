function H=Harmonic(t,b,n)
% Integral_0^t sin(u)*tan(b*sin(n*u)) du, with exact period symmetries.
shape=size(t);t=mod(t(:),2*pi);
% H.m supplies the nonzero complete-period integral for the n=1 endpoint.
t=reshape(t,[],1);[z,w]=sinsteer.Gauss(24);quarter=pi/2;
rawQuarter=quarter*(sin(quarter*z).*tan(b*sin(n*quarter*z)))'*w;
sector=min(3,floor(t/quarter));x=t-sector*quarter;x(mod(sector,2)==1)=quarter-x(mod(sector,2)==1);
u=x.*z';h=x.*((sin(u).*tan(b*sin(n*u)))*w);
if n==1
 H=h;H(sector==1)=2*rawQuarter-h(sector==1);H(sector==2)=2*rawQuarter+h(sector==2);H(sector==3)=4*rawQuarter-h(sector==3);
else
 H=h;H(sector>=2)=-h(sector>=2);
end
H=reshape(H,shape);
end
