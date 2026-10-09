function piece=Curve(A,B,eta,gear)
% Simplified eta3 polynomial, equivalent to the coefficients in Section III.
% A,B = [x,y,physical_heading,physical_curvature,geometric_curvature_derivative].
angles=[A(3),B(3)]+(gear<0)*pi;
tangent=[cos(angles(:)),sin(angles(:))];normal=[-tangent(:,2),tangent(:,1)];
curvature=gear*[A(4);B(4)];derivative=[A(5);B(5)];eta=eta(:);
first=eta.*tangent;second=eta.^2.*curvature.*normal;third=eta.^3.*derivative.*normal;
C=zeros(8,2);C(1:4,:)=[A(1:2);first(1,:);second(1,:)/2;third(1,:)/6];
right=[B(1:2)-sum(C(1:4,:));first(2,:)-[0 1 2 3]*C(1:4,:);second(2,:)-[0 0 2 6]*C(1:4,:);third(2,:)-6*C(4,:)];
M=[1 1 1 1;4 5 6 7;12 20 30 42;24 60 120 210];C(5:8,:)=M\right;
piece=struct('coefficients',C,'gear',gear,'length',NaN,'parameter_nodes',[],'mileage_nodes',[]);
end
