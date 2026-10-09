function next=Step(state,control,h,lw)
% Equation (7): one classical RK4 step with a constant (a,phi) input.
k1=flow(state,control,lw);k2=flow(state+h/2*k1,control,lw);
k3=flow(state+h/2*k2,control,lw);k4=flow(state+h*k3,control,lw);
next=state+h/6*(k1+2*k2+2*k3+k4);
end
function dz=flow(z,u,lw)
dz=[z(:,4).*cos(z(:,3)),z(:,4).*sin(z(:,3)),z(:,4).*tan(u(:,2))/lw,u(:,1)];
end
