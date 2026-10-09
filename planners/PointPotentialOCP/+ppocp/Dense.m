function [q,derivative,interval]=Dense(z,vehicle,times)
% Cubic Hermite physical poses and piecewise-linear speed/steering.
N=(numel(z)-1)/5;states=reshape(z(1:end-1),5,N);T=z(end);knots=linspace(0,T,N);h=T/(N-1);times=times(:)';
interval=discretize(times,knots);assert(all(isfinite(interval)),'Dense query lies outside the native time horizon.');s=min(1,max(0,(times-knots(interval))/h));
f=ppocp.Model(states,vehicle.lw);a=states(:,interval);b=states(:,interval+1);fa=f(:,interval);fb=f(:,interval+1);
q=a+(b-a).*s;q(1:3,:)=(2*s.^3-3*s.^2+1).*a(1:3,:)+(s.^3-2*s.^2+s)*h.*fa+(-2*s.^3+3*s.^2).*b(1:3,:)+(s.^3-s.^2)*h.*fb;
derivative=(b-a)/h;derivative(1:3,:)=(6*s.^2-6*s)/h.*a(1:3,:)+(3*s.^2-4*s+1).*fa+(-6*s.^2+6*s)/h.*b(1:3,:)+(3*s.^2-2*s).*fb;
end
