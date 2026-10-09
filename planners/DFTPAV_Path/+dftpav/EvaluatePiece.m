function [pose,speed,native]=EvaluatePiece(C,u,eta,h,vehicle)
xy=dftpav.Basis(u,0)*C;d=dftpav.Basis(u,1)*C;dd=dftpav.Basis(u,2)*C;ddd=dftpav.Basis(u,3)*C;
speed=vecnorm(d,2,2);cross=d(:,1).*dd(:,2)-d(:,2).*dd(:,1);curvature=eta*cross./speed.^3;
theta=atan2(eta*d(:,2),eta*d(:,1));pose=[xy theta curvature];
if nargout>2
 kp=eta*((d(:,1).*ddd(:,2)-d(:,2).*ddd(:,1))./speed.^3-3*cross.*sum(d.*dd,2)./speed.^5)/h;
 native=struct('v',eta*speed/h,'a',eta*sum(d.*dd,2)./(speed*h^2),'phi',atan(vehicle.lw*curvature),'omega',vehicle.lw*kp./(1+(vehicle.lw*curvature).^2));
end
end
