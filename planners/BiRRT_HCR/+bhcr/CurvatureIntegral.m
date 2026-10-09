function total=CurvatureIntegral(u)
% Exact integral of absolute piecewise quadratic curvature.
total=0;
for j=1:size(u,1)
 L=abs(u(j,1));k=u(j,2);s=u(j,3);r=u(j,4);
 rootsHere=roots([.5*r,s,k]);rootsHere=real(rootsHere(abs(imag(rootsHere))<1e-10));
 z=sort([0;rootsHere(rootsHere>0&rootsHere<L);L]);
 primitive=k*z+.5*s*z.^2+r*z.^3/6;total=total+sum(abs(diff(primitive)));
end
end
