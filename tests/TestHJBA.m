function report=TestHJBA()
% Independent analytic HJ solutions, convex QPs and exact branch joins.
grid=struct('x',linspace(-2,2,9),'y',linspace(-2,2,7),'theta',(0:30)*2*pi/31);[X,Y,A]=ndgrid(grid.x,grid.y,grid.theta); %#ok<ASGLU>
[value,~]=hjba.ReachTube(X,grid,-1,0,.8,.4);exact=X-.8*max(cos(A),0);transport=max(abs(value-exact),[],'all');assert(transport<1e-11);
errors=zeros(1,3);compiledError=nan;
for j=1:3
 n=[31 61 121];n=n(j);grid=struct('x',[-1 0 1],'y',[-1 0 1],'theta',(-floor(n/2):floor(n/2))*2*pi/n);
 [~,~,A]=ndgrid(grid.x,grid.y,grid.theta);target=1-cos(A)-(1-cos(.3));[value,~]=hjba.ReachTube(target,grid,0,.4,1,.4);
 exact=1-cos(max(abs(A)-.4,0))-(1-cos(.3));rows=abs(A)<2.5;errors(j)=max(abs(value(rows)-exact(rows)));
end
assert(errors(3)<.006&&errors(3)<errors(1)/3);
if exist('hjba_hj_mex','file')==3
 grid=struct('x',linspace(-2,2,11),'y',linspace(-1,3,9),'theta',(0:16)*2*pi/17);[X,Y,A]=ndgrid(grid.x,grid.y,grid.theta);target=max(max(abs(X),abs(Y)),1-cos(A))-.2;
 native=hjba.ReachTube(target,grid,-1,.3,.8,.4);reference=hjba.ReachTube(target,grid,-1,.3,.8,.4,true);compiledError=max(abs(native-reference),[],'all');assert(compiledError<1e-10);
end
c=LoadCase(3);p=parking.PolygonData(c);v=c.vehicle;old=rng;rng(1826);restore=onCleanup(@()rng(old)); %#ok<NASGU>
agreement=0;QP=optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'ConstraintTolerance',1e-10);
for k=1:30
 q=[-5+10*rand,-3+10*rand,-pi+2*pi*rand];body=parking.VehiclePolygon(q,v);body=body(1:4,:);edge=body([2:4 1],:)-body;B=[edge(:,2),-edge(:,1)];B=B./vecnorm(B,2,2);b=sum(B.*body,2);safe=true;
 for j=1:p.count
  H=2*[eye(2),-eye(2);-eye(2),eye(2)];[~,cost,flag]=quadprog(H,zeros(4,1),blkdiag(B,p.A{j}),[b;p.b{j}],[],[],[],[],[],QP);assert(flag>0);safe=safe&&(cost>1e-9);
 end
 native=parking.FootprintClearance(q,c,0);assert(native==safe);agreement=agreement+1;
end
c.task.x0=-2;c.task.y0=0;c.task.theta0=.1;c.task.xf=2;c.task.yf=0;c.task.thetaf=-.1;c.obstacle.num_obs=0;c.obstacle.obs={};o=hjba.Config();o.branchSeconds=5;
branch=hjba.Branch(c,[0 2 .5],[-10 -10;10 10],o);assert(branch.success&&all(branch.expanded==1));
q=[c.task.x0 c.task.y0 c.task.theta0];for k=1:size(branch.primitives,1),q=parking.IntegratePrimitive(q,branch.primitives(k,1),branch.primitives(k,2));end
error=q-[c.task.xf c.task.yf c.task.thetaf];error(3)=atan2(sin(error(3)),cos(error(3)));assert(max(abs(error))<1e-9);
report=struct('passed',true,'linear_HJ_error',transport,'angular_HJ_refinement_errors',errors,'compiled_MATLAB_difference',compiledError,'SAT_QP_agreements',agreement,'bidirectional_RS_endpoint_error',max(abs(error)),'curvature_continuity_claimed',false);disp(report);
end
