function next=Step(z,u,h,lw)
k1=flow(z,u,lw);k2=flow(z+h/2*k1,u,lw);k3=flow(z+h/2*k2,u,lw);k4=flow(z+h*k3,u,lw);
next=z+h/6*(k1+2*k2+2*k3+k4);
end
function dz=flow(z,u,lw)
dz=[z(:,4).*cos(z(:,3)),z(:,4).*sin(z(:,3)),z(:,4).*tan(z(:,5))/lw,u];
end
