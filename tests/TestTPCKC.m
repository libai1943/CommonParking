function report=TestTPCKC()
SetupCommonParking();c=LoadCase(1);poly=parking.PolygonData(c);old=rng;restore=onCleanup(@()rng(old));rng(154); %#ok<NASGU>
maximumGap=0;settings=optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'ConstraintTolerance',1e-10);
for i=1:min(3,poly.count)
 A=poly.A{i};b=poly.b{i};p=poly.vertices{i};
 for j=1:10
  query=mean(p)+8*rand(1,2)-4;[lambda,certificate]=tpckc.PointDual(query,A,b,p);
  [projection,~,flag]=quadprog(eye(2),-query',A,b,[],[],[],[],[],settings);assert(flag>0);distance=norm(query'-projection);
  maximumGap=max(maximumGap,abs(distance-(A*query'-b)'*lambda));assert(certificate.dual_norm<=1+1e-8&&maximumGap<1e-5);
 end
end
% A plus-shaped intersection has no vertex intrusion in either direction.
x=(c.vehicle.lw+c.vehicle.lf-c.vehicle.lr)/2;p=[x-.1 -3;x+.1 -3;x+.1 3;x-.1 3;x-.1 -3];c.obstacle.num_obs=1;c.obstacle.obs={struct('x',p(:,1)','y',p(:,2)')};
q=struct('x',zeros(5,1),'y',zeros(5,1),'theta',zeros(5,1));assert(isempty(tpckc.Collect(q,c,0)));assert(~parking.FootprintClearance([0 0 0],c,0));
% Propagation replicates keys only to interior time nodes.
p=[3.7 .9;3.9 .9;3.9 1.1;3.7 1.1;3.7 .9];c.obstacle.obs={struct('x',p(:,1)','y',p(:,2)')};q.y=[10;10;0;10;10];base=tpckc.Collect(q,c,0);spread=tpckc.Collect(q,c,2);assert(~isempty(base)&&all(base(:,1)==3)&&isequal(unique(spread(:,1)),(2:4)'));
report=struct('passed',true,'independent_projection_dual_gap',maximumGap,'vertex_crossing_counterexample',true,'propagation_excludes_endpoints',true);disp(report);
end
