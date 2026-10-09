function [A,B]=Shape(beta,gamma)
% Kiss/Tevesz Eqs. (41)--(45), evaluated by 16-point Gauss integration.
% Factored half-angle form avoids subtracting cosines for small turns.
if nargin<2,gamma=0;end
beta=beta(:);gamma=gamma+zeros(size(beta));
persistent z w
if isempty(z)
 j=(1:15)';b=j./sqrt(4*j.^2-1);[V,D]=eig(diag(b,1)+diag(b,-1));[z,ix]=sort(diag(D));w=2*V(1,ix)'.^2;z=(z+1)/2;w=w/2;
end
angle=abs(beta).*z'.^2/2;X=beta.*(cos(angle)*w);Y=abs(beta).*(sin(angle)*w);
h=(beta+gamma)/2;chord=X.*cos(h)+Y.*sin(h)+sin(gamma/2);
A=2*cos(h).*chord;B=2*sin(h).*chord;
end
