function u=Parameter(piece,s)
s=max(0,min(piece.length,s(:)));u=interp1(piece.mileage_nodes,piece.parameter_nodes,s);
bin=discretize(s,[-Inf;piece.mileage_nodes(2:end-1);Inf]);left=piece.parameter_nodes(bin);right=piece.parameter_nodes(bin+1);
base=piece.mileage_nodes(bin);
for iteration=1:6
 distance=zeros(size(s));for k=1:numel(s),distance(k)=eta3.IntegrateLength(piece,[left(k);u(k)]);end
 [~,speed]=eta3.Evaluate(piece,u);u=max(left,min(right,u-(base+distance-s)./max(speed,realmin)));
end
u(s==0)=0;u(s==piece.length)=1;
end
