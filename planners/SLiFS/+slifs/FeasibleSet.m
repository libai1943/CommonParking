function [A,b,minimumGap]=FeasibleSet(z,c,options)
% Eqs. (11)-(13): all circle positions are affine in P and the last centre.
N=options.nodes;D=7*N+1;q=reshape(z(1:7*N),N,7);polygons=parking.PolygonData(c);
discs=cp.CoveringDiscs(c.vehicle,options.circles,1);ratio=discs.offsets(:,1)/discs.offsets(end,1);
rows=zeros(0,1);columns=rows;values=rows;b=zeros(0,1);minimumGap=inf;
for circle=1:options.circles
 fraction=ratio(circle);points=(1-fraction)*q(:,1:2)+fraction*q(:,6:7);
 for obstacle=1:polygons.count
  [normal,support,distance]=slifs.Support(points,polygons.vertices{obstacle});minimumGap=min(minimumGap,min(distance-discs.radius-options.clearance));
  r=numel(b)+(1:N)';b=[b;-support-discs.radius-options.clearance]; %#ok<AGROW>
  cols=[(1:N)',N+(1:N)',5*N+(1:N)',6*N+(1:N)'];vals=-[normal*(1-fraction),normal*fraction];
  rows=[rows;repmat(r,4,1)];columns=[columns;cols(:)];values=[values;vals(:)]; %#ok<AGROW>
 end
end
A=sparse(rows,columns,values,numel(b),D);
end
