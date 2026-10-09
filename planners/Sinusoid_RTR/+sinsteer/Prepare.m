function p=Prepare(p)
% Exact prescribed-input quadrature on each quarter period.
if p.n==0,p.increments=[p.a*p.alpha/sqrt(1-p.alpha^2),abs(p.a)/sqrt(1-p.alpha^2)];p.length=p.increments(2)*p.scale;return;end
[z,w]=sinsteer.Gauss(32);p.increments=zeros(4,2);
for j=1:4,t=(j-1)*pi/2+pi/2*z;[dy,ds]=sinsteer.Integrand(p,t);p.increments(j,:)=(pi/2*[dy ds]'*w)';end
p.length=sum(p.increments(:,2))*p.scale;
end
