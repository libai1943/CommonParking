function g=Geometry(coeff,s,L,gear,wheelbase)
degree=size(coeff,1)-1;p=ritp.Basis(s,L,degree,0)*coeff;
d=ritp.Basis(s,L,degree,1)*coeff;dd=ritp.Basis(s,L,degree,2)*coeff;
ddd=ritp.Basis(s,L,degree,3)*coeff;speed=hypot(d(:,1),d(:,2));
cross=d(:,1).*dd(:,2)-d(:,2).*dd(:,1);
crossDerivative=d(:,1).*ddd(:,2)-d(:,2).*ddd(:,1);
dot=sum(d.*dd,2);curvature=gear*cross./speed.^3;
curvatureDerivative=gear*(crossDerivative./speed.^3-3*cross.*dot./speed.^5);
g=struct('x',p(:,1),'y',p(:,2),'theta',unwrap(atan2(gear*d(:,2),gear*d(:,1))), ...
 'phi',atan(wheelbase*curvature),'phi_s',wheelbase*curvatureDerivative./(1+(wheelbase*curvature).^2), ...
 'parameter_speed',speed,'curvature',curvature);
end
