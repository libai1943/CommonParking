function report=TestLamirauxSmooth()
c=LoadCase(1);c.obstacle.num_obs=0;c.obstacle.obs={};o=lamiraux.Config();state=rng;clean=onCleanup(@()rng(state));rng(2001); %#ok<NASGU>
derivativeError=0;boundTests=0;inverseError=0;regular=0;
for trial=1:80
 a=[randn(1,2),randn,.2*(2*rand-1)];b=[3*randn(1,2),randn,.2*(2*rand-1)];v=sign(rand-.5)*(.5+5*rand);p=lamiraux.Segment(a,b,v,'blend');
 t=.1+.8*rand(20,1);eps=1e-5;[~,~,first,second,third]=lamiraux.Evaluate(p,t);[qp,~,fp,sp]=lamiraux.Evaluate(p,t+eps);[qm,~,fm,sm]=lamiraux.Evaluate(p,t-eps);
 derivativeError=max(derivativeError,max(abs([(qp(:,1:2)-qm(:,1:2))/(2*eps)-first;(fp-fm)/(2*eps)-second;(sp-sm)/(2*eps)-third]),[],'all'));
 left=sort(rand(20,1));right=min(1,left+.001+.05*rand(20,1));[lo,hi,kap]=lamiraux.DerivativeBounds(p,left,right);
 for interval=1:20
  [q,speed]=lamiraux.Evaluate(p,linspace(left(interval),right(interval),51)');
  assert(all(speed>=lo(interval)-1e-9)&&all(speed<=hi(interval)+1e-9));assert(all(abs(q(:,4))<=kap(interval)+1e-8));boundTests=boundTests+1;
 end
 if lamiraux.Certify(p,c,o,false)
  regular=regular+1;p=lamiraux.Prepare(p);mileage=p.length*rand(15,1);parameter=lamiraux.Parameter(p,mileage);
  for j=1:numel(mileage),actual=integral(@(u)speedOnly(p,u),0,parameter(j),'AbsTol',1e-10,'RelTol',1e-10);inverseError=max(inverseError,abs(actual-mileage(j)));end
 end
end
% Regular curved and reverse blends close to canonical arcs, for inversion.
for trial=1:20
 a=[randn(1,2),randn,.1*(2*rand-1)];v=sign(rand-.5)*(2+3*rand);b=[parking.IntegratePrimitive(a(1:3),v,a(4)),a(4)];b=b+[.01*randn(1,2),.005*randn,.005*randn];
 p=lamiraux.Segment(a,b,v,'blend');assert(lamiraux.Certify(p,c,o,false));regular=regular+1;p=lamiraux.Prepare(p);mileage=p.length*rand(10,1);parameter=lamiraux.Parameter(p,mileage);
 for j=1:numel(mileage),actual=integral(@(u)speedOnly(p,u),0,parameter(j),'AbsTol',1e-10,'RelTol',1e-10);inverseError=max(inverseError,abs(actual-mileage(j)));end
end
assert(regular>=20&&derivativeError<2e-5&&inverseError<1e-7);
inputs={ {[0 0 0 0],[5 1 .2 0]}, {[0 0 0 0],[0 .1 0 0]}, {[0 0 0 .1],[4 1 .3 -.1]}, {[0 0 .3 0],[-4 -1 .1 .05]} };
for pair=inputs
 input=pair{1};[pieces,ok]=lamiraux.Steer(input{1},input{2},c,o);assert(ok);
 endpoints=[lamiraux.Evaluate(pieces(1),0);lamiraux.Evaluate(pieces(end),1)];assert(max(abs(endpoints-[input{1};input{2}]),[],'all')<1e-8);
 for j=1:numel(pieces)-1,a=lamiraux.Evaluate(pieces(j),1);b=lamiraux.Evaluate(pieces(j+1),0);assert(max(abs(a-b))<1e-8);end
end
% Collision between free endpoint footprints must be rejected.
p=lamiraux.Segment([0 0 0 0],[10 0 0 0],10,'blend');c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[5.99 6.01 6.01 5.99 5.99],'y',[-3 -3 3 3 -3])};
assert(parking.FootprintClearance([0 0 0;10 0 0],c,0));assert(~lamiraux.Certify(p,c,o,true));
% Length inversion on a guaranteed regular smooth curve.
p=lamiraux.Prepare(lamiraux.Segment([0 0 0 0],[5 .2 .1 0],5,'blend'));mileage=linspace(0,p.length,101)';parameter=lamiraux.Parameter(p,mileage);
for j=1:numel(mileage),actual=integral(@(u)speedOnly(p,u),0,parameter(j),'AbsTol',1e-10,'RelTol',1e-10);inverseError=max(inverseError,abs(actual-mileage(j)));end
assert(inverseError<1e-7);
report=struct('passed',true,'random_blends',80,'derivative_error',derivativeError,'interval_bound_tests',boundTests,'regular_random_blends',regular,'arc_length_inversion_error',inverseError,'thin_barrier_rejected',true);disp(report);
end
function speed=speedOnly(p,t),shape=size(t);[~,speed]=lamiraux.Evaluate(p,t);speed=reshape(speed,shape);end
