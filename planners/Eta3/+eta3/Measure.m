function [cost,inequality,values]=Measure(z,c,gears,o)
pieces=eta3.Decode(z,c.task,gears);u=linspace(0,1,o.intervals+1)';
maxKappa=0;maxDerivative=0;length=0;inequality=zeros(0,1);
for piece=pieces
 [q,speed,derivative]=eta3.Evaluate(piece,u);
 maxKappa=max(maxKappa,max(abs(q(:,4))));maxDerivative=max(maxDerivative,max(abs(derivative)));
 length=length+sum(eta3.IntegrateLength(piece,linspace(0,1,13)'));
 [~,gap]=parking.FootprintClearance(q(:,1:3),c,0);
 gap(isinf(gap))=1e6;
 inequality=[inequality;abs(q(:,4))-c.vehicle.kappa_max;abs(derivative)-o.curvatureDerivativeMax; ...
  o.clearance-gap;o.minimumSpeed-speed]; %#ok<AGROW>
end
values=[maxKappa,maxDerivative,length];cost=o.weights*values';
end
