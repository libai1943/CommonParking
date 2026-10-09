function [maximumCurvature,minimumSpeed]=Extrema(coefficients)
% Stationary curvature and speed of an individual cubic polynomial span.
px=flipud(coefficients(:,1))';py=flipud(coefficients(:,2))';dx=polyder(px);dy=polyder(py);ddx=polyder(dx);ddy=polyder(dy);
N=conv(dx,ddy)-conv(dy,ddx);V=conv(dx,dx)+conv(dy,dy);
stationary=subtract(conv(polyder(N),V),1.5*conv(N,polyder(V)));
events=unique([0;1;insideRoots(stationary)]);speedEvents=[0;1;insideRoots(polyder(V))];
maximumCurvature=max(abs(polyval(N,events))./max(realmin,polyval(V,events)).^1.5);minimumSpeed=sqrt(max(0,min(polyval(V,speedEvents))));
maximumCurvature=maximumCurvature+1e-10*(1+maximumCurvature);minimumSpeed=max(0,minimumSpeed-1e-10*(1+minimumSpeed));
end
function result=subtract(a,b)
n=max(numel(a),numel(b));result=[zeros(1,n-numel(a)),a]-[zeros(1,n-numel(b)),b];
end
function r=insideRoots(p)
first=find(abs(p)>1e-14*max(abs(p)),1);if isempty(first),r=zeros(0,1);return;end
r=roots(p(first:end));r=real(r(abs(imag(r))<1e-8&real(r)>0&real(r)<1));
end
