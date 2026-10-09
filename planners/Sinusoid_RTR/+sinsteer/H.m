function H=H(t,b,n)
% Continuous closed-period antiderivative in the normalized vehicle chart.
raw=t;H=sinsteer.Harmonic(t,b,n);
if n==1
 [z,w]=sinsteer.Gauss(24);q=pi/2;Q=q*(sin(q*z).*tan(b*sin(q*z)))'*w;H(abs(raw-2*pi)<1e-12)=4*Q;
else,H(abs(raw-2*pi)<1e-12)=0;end
end
