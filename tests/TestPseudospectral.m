function report = TestPseudospectral()
[tau,D,w,b]=cp.LegendreLobatto(15);error=0;
for k=0:14
    dy=zeros(15,1);if k>0,dy=k*tau.^(k-1);end
    error=max(error,max(abs(D*tau.^k-dy)));
end
assert(error<1e-10);integrationError=0;
for k=0:27
    exact=0;if mod(k,2)==0,exact=2/(k+1);end
    integrationError=max(integrationError,abs(w'*tau.^k-exact));
end
assert(integrationError<1e-12);
q=linspace(-1,1,121)';z=cp.BarycentricInterpolate(tau,b,[tau.^3 tau.^14],q);
interpolationError=max(abs(z-[q.^3 q.^14]),[],'all');assert(interpolationError<1e-12);
report=struct('passed',true,'derivative_error',error,'quadrature_error',integrationError,'interpolation_error',interpolationError);disp(report);
end
