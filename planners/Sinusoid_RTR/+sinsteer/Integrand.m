function [dy,ds]=Integrand(p,t)
alpha=p.alpha+p.a*sinsteer.H(t,p.b,p.n);assert(all(abs(alpha)<1,'all'),'Sinusoid_RTR:Chart','Sinusoidal path left its nonsingular chart.');
vx=p.a*sin(t);dy=vx.*alpha./sqrt(1-alpha.^2);ds=abs(vx)./sqrt(1-alpha.^2);
end
