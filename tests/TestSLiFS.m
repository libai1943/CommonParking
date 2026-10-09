function report=TestSLiFS()
% Independent directional derivatives and convex support geometry checks.
cfg=BenchmarkConfig();v=cfg.vehicle;N=17;prior=rng;clean=onCleanup(@()rng(prior));rng(177);
q=randn(N,7);q(:,5)=.3*q(:,5);z=[q(:);12];[g,A]=slifs.Residual(z,N,v,3); %#ok<ASGLU>
directions=randn(numel(z),15);error=0;
for j=1:size(directions,2)
 d=directions(:,j);eps=1e-6;numeric=(slifs.Residual(z+eps*d,N,v,3)-slifs.Residual(z-eps*d,N,v,3))/(2*eps);
 error=max(error,max(abs(numeric-A*d)));
end
assert(error<1e-7);
[J,H,f]=slifs.Objective(z,N);assert(abs(J-(.5*z'*H*z+f'*z))<1e-10);
polygon=[-2 -1;2 -1;2 1;-2 1];points=6*randn(500,2);[normal,offset,distance]=slifs.Support(points,polygon);
supportViolation=max(normal*polygon'-offset,[],'all');assert(supportViolation<1e-10);
expected=hypot(max(abs(points(:,1))-2,0),max(abs(points(:,2))-1,0));inside=abs(points(:,1))<2&abs(points(:,2))<1;
expected(inside)=-min(2-abs(points(inside,1)),1-abs(points(inside,2)));assert(max(abs(expected-distance))<1e-12);
c=LoadCase(1);c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[5.9 6.1 6.1 5.9 5.9],'y',[-3 -3 3 3 -3])};
assert(parking.FootprintClearance([0 0 0;10 0 0],c,0));assert(slifs.SweptCollision([0 0 0;10 0 0],c));
assert(~slifs.SweptCollision([0 10 0;10 10 0],c));
report=struct('passed',true,'jacobian_error',error,'support_violation',supportViolation,'distance_error',max(abs(expected-distance)));disp(report);
end
