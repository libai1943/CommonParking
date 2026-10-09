function [next,cost]=Step(state,u,h,gear,vehicle,gamma)
% Exact steering propagation; RK4 of position/heading with prescribed alpha(s).
alpha=state(:,4);omega=state(:,5);middle=alpha+.5*h.*omega+.125*h.^2.*u;
last=alpha+h.*omega+.5*h.^2.*u;
k1=gear.*tan(alpha)/vehicle.lw;k2=gear.*tan(middle)/vehicle.lw;k4=gear.*tan(last)/vehicle.lw;theta=state(:,3);
angles=[theta,theta+.5*h.*k1,theta+.5*h.*k2,theta+h.*k2];
next=state;next(:,1:2)=state(:,1:2)+gear.*h/6.*[cos(angles)*[1;2;2;1],sin(angles)*[1;2;2;1]];
next(:,3)=theta+h/6.*(k1+4*k2+k4);next(:,4)=last;next(:,5)=omega+h.*u;
cost=h.*(1+gamma*(u.^2+(alpha.^2+4*middle.^2+last.^2+10*(omega.^2+4*(omega+.5*h.*u).^2+next(:,5).^2))/6));
end
