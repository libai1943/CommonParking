function u=Turn(beta,kappa,sigma)
% Symmetric continuous-curvature turn, optionally reshaped from E to T.
% Controls: signed length, initial curvature, d(curvature)/d(unsigned mileage).
if abs(beta)<1e-14,u=zeros(0,3);return;end
assert(abs(kappa)>0);ratio=0;
if nargin==3
 [~,B0]=rtr.Shape(beta);lo=0;hi=1-1e-12;
 assert(kappa^2/abs(beta)<=sigma*(1+1e-9));
 for k=1:48
  mid=(lo+hi)/2;[~,B]=rtr.Shape((1-mid)*beta,mid*beta);km=kappa*B/B0;
  if km^2/abs((1-mid)*beta)<=sigma,lo=mid;else,hi=mid;end
 end
 ratio=lo;[~,B]=rtr.Shape((1-ratio)*beta,ratio*beta);kappa=kappa*B/B0;
end
b=(1-ratio)*beta;g=ratio*beta;h=abs(b/kappa);d=sign(beta/kappa);
u=[d*h,0,kappa/h];
if abs(g)>1e-14,u=[u;d*abs(g/kappa),kappa,0];end
u=[u;d*h,kappa,-kappa/h];
end
