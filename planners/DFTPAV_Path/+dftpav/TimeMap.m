function [T,dT]=TimeMap(tau)
% Paper equations (74)-(76).
T=zeros(size(tau));dT=T;positive=tau>0;
T(positive)=.5*tau(positive).^2+tau(positive)+1;dT(positive)=tau(positive)+1;
den=tau(~positive).^2-2*tau(~positive)+2;T(~positive)=2./den;dT(~positive)=4*(1-tau(~positive))./den.^2;
end
