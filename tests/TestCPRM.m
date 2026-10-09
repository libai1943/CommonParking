function report=TestCPRM()
state=rng;clean=onCleanup(@()rng(state));rng(112);endpointError=0;boundGap=inf; %#ok<NASGU>
for j=1:200
 p=3*randn(1,2);angle=2*pi*rand;beta=.1+(pi-.2)*rand;a=p+(.2+5*rand)*[cos(angle) sin(angle)];b=p+(.2+5*rand)*[cos(angle+beta) sin(angle+beta)];direction=sign(rand-.5);
 [pp,start,goal]=cprm.Fillet(a,b,p,direction);q=cp.ArcPose(start,pp,sum(abs(pp(:,1))));err=q-goal;err(3)=atan2(sin(err(3)),cos(err(3)));endpointError=max(endpointError,max(abs(err)));
 assert(all(sign(pp(:,1))==direction));
end
assert(endpointError<1e-8);
for count=4:10
 control=cumsum(randn(count,2),1);spans=count-3;width=.2+rand(1,spans);breaks=[0 cumsum(width)/sum(width)];knots=[zeros(1,3),breaks,ones(1,3)];
 B=cprm.Basis(linspace(0,1,301)',knots,3);assert(max(abs(sum(B,2)-1))<1e-12&&min(B,[],'all')>=0);
 pieces=cprm.SplinePieces(control,knots,1);
 for j=1:numel(pieces)
  t=linspace(0,1,1001)';[q,speed]=cprm.Evaluate(pieces(j),t);direct=cprm.Basis(breaks(j)+t*(breaks(j+1)-breaks(j)),knots,3)*control;
  assert(max(abs(q(:,1:2)-direct),[],'all')<1e-9);assert(max(abs(q(:,4)))<=pieces(j).maximum_curvature+1e-7);assert(min(speed)>=pieces(j).minimum_speed-1e-9);
  boundGap=min(boundGap,pieces(j).maximum_curvature-max(abs(q(:,4))));
  if j<numel(pieces)
   next=cprm.Evaluate(pieces(j+1),0);error=q(end,:)-next;error(3)=atan2(sin(error(3)),cos(error(3)));assert(max(abs(error))<1e-7);
  end
 end
end
c=LoadCase(1);c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[5.99 6.01 6.01 5.99 5.99],'y',[-3 -3 3 3 -3])};p=cprm.Piece('cubic',1);p.coefficients=[0 0;10 0;0 0;0 0];[p.maximum_curvature,p.minimum_speed]=cprm.Extrema(p.coefficients);assert(~cprm.CubicFree(p,c,cprm.Config()));
report=struct('passed',true,'fillet_connections',200,'maximum_fillet_endpoint_error',endpointError,'spline_span_curvature_bound_gap',boundGap,'thin_barrier_rejected',true);disp(report);
end
