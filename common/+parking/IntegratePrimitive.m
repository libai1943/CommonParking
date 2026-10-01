function q = IntegratePrimitive(q0,s,kappa)
% Exact kinematic bicycle propagation over SIGNED rear-axle arc length.
s=s(:); t=q0(3)+kappa*s;
if abs(kappa)<1e-12
    q=[q0(1)+s*cos(q0(3)),q0(2)+s*sin(q0(3)),t];
else
    q=[q0(1)+(sin(t)-sin(q0(3)))/kappa, ...
       q0(2)-(cos(t)-cos(q0(3)))/kappa,t];
end
q(:,3)=atan2(sin(q(:,3)),cos(q(:,3)));
end
