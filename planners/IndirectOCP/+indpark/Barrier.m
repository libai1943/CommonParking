function [b,g,h]=Barrier(x,limit,mu)
% Strict logarithmic regularization of a symmetric physical bound.
d=limit.^2-x.^2;b=-mu*log(d./limit.^2);g=2*mu*x./d;h=2*mu*(limit.^2+x.^2)./d.^2;
end
