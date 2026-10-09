function piece=Prepare(piece)
previous=NaN;
for count=2.^(4:11)
 u=linspace(0,1,count+1)';lengths=eta3.IntegrateLength(piece,u);total=sum(lengths);
 if abs(total-previous)<1e-10*(1+total),break;end
 previous=total;
end
piece.length=total;piece.parameter_nodes=u;piece.mileage_nodes=[0;cumsum(lengths)];
end
