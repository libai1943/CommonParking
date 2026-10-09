function report=TestEta3()
SetupCommonParking();original=rng;cleanup=onCleanup(@()rng(original));rng(174); %#ok<NASGU>
endpointError=0;derivativeError=0;
for trial=1:100
 A=[randn(1,2),randn,.1*randn,.1*randn];B=[randn(1,2),randn,.1*randn,.1*randn];
 eta=1+3*rand(1,2);gear=sign(randn);piece=eta3.Curve(A,B,eta,gear);
 [q,speed,dk]=eta3.Evaluate(piece,[0;1]);angle=atan2(sin(q(:,3)-[A(3);B(3)]),cos(q(:,3)-[A(3);B(3)]));
 errors=[q(:,1:2)-[A(1:2);B(1:2)],angle,q(:,4)-[A(4);B(4)],dk-[A(5);B(5)],speed-eta'];
 endpointError=max(endpointError,max(abs(errors),[],'all'));
 u=.1+.8*rand(1);h=1e-6;[~,~,d,first]=eta3.Evaluate(piece,u);plus=eta3.Evaluate(piece,u+h);minus=eta3.Evaluate(piece,u-h);
 derivativeError=max(derivativeError,abs(d-gear*(plus(4)-minus(4))/(2*h*norm(first)))/max(1,abs(d)));
end
assert(endpointError<1e-7&&derivativeError<1e-5);
o=eta3.Config();c=LoadCase(1);c.obstacle.num_obs=0;c.obstacle.obs={};c.task=struct('x0',0,'y0',0,'theta0',0,'xf',3,'yf',0,'thetaf',0);
z0=[0;0;3;3];lower=[-2.5;-2.5;.01;.01];upper=[2.5;2.5;20;20];
[z,info]=eta3.Optimize(z0,lower,upper,c,1,o);assert(info.success);assert(abs(info.objectives(3)-3)<1e-6);
piece=eta3.Prepare(eta3.Decode(z,c.task,1));s=linspace(0,piece.length,101)';u=eta3.Parameter(piece,s);q=eta3.Evaluate(piece,u);
assert(max(abs(q(:,1)-s))<1e-7);report=struct('passed',true,'random_endpoint_tests',100, ...
 'maximum_endpoint_error',endpointError,'curvature_derivative_relative_error',derivativeError,'straight_nlp_exitflag',info.exitflag);disp(report);
end
