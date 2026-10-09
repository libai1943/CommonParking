function [q,speed,curvatureDerivative,first]=Evaluate(piece,u)
u=u(:);C=piece.coefficients;P=poly(u,C);
first=poly(u,(1:7)'.*C(2:end,:));second=poly(u,((2:7)'.*(1:6)').*C(3:end,:));
third=poly(u,((3:7)'.*(2:6)'.*(1:5)').*C(4:end,:));
v2=sum(first.^2,2);speed=sqrt(v2);safe=max(v2,1e-24);
cross=first(:,1).*second(:,2)-first(:,2).*second(:,1);
kappa=cross./safe.^(3/2);
curvatureDerivative=(first(:,1).*third(:,2)-first(:,2).*third(:,1))./safe.^2-3*cross.*sum(first.*second,2)./safe.^3;
q=[P,atan2(piece.gear*first(:,2),piece.gear*first(:,1)),piece.gear*kappa];
end
function value=poly(u,C)
value=repmat(C(end,:),numel(u),1);
for j=size(C,1)-1:-1:1,value=value.*u+C(j,:);end
end
