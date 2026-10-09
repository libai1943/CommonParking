function [u,d,b]=Control(lambda,limit,mu)
% Solve lambda + derivative(-mu*log(1-u^2/limit^2))=0 exactly.
s=sqrt(mu^2+(limit.*lambda).^2);u=-limit.^2.*lambda./(mu+s);
[b,~,h]=indpark.Barrier(u,limit,mu);d=-1./h;
end
