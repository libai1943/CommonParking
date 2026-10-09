function report=TestBIAGT()
% Independent curve integration and discrete search invariants.
biagt.EnsureNative();c=LoadCase(3);v=c.vehicle;o=biagt.Config(v);rng(2024096);
Q=[10*randn(300,2),6*pi*randn(300,1)];P=[10*randn(300,2),6*pi*randn(300,1)];[L,edges]=biagt_rs_mex(Q,P,v.kappa_max);reverse=biagt_rs_mex(P,Q,v.kappa_max);endpoint=0;odeError=0;
for j=1:300
 q=Q(j,:);for k=1:size(edges{j},1),q=biagt.Advance(q,edges{j}(k,1),edges{j}(k,2));end
 endpoint=max(endpoint,max(abs([q(1:2)-P(j,1:2),atan2(sin(q(3)-P(j,3)),cos(q(3)-P(j,3)))])));
 rev=flipud(edges{j});rev(:,1)=-rev(:,1);q=P(j,:);for k=1:size(rev,1),q=biagt.Advance(q,rev(k,1),rev(k,2));end
 endpoint=max(endpoint,max(abs([q(1:2)-Q(j,1:2),atan2(sin(q(3)-Q(j,3)),cos(q(3)-Q(j,3)))])));
 assert(abs(sum(abs(edges{j}(:,1)))-L(j))<1e-10);
 if j<=30
  q=Q(j,:);
  for k=1:size(edges{j},1)
   len=edges{j}(k,1);curvature=edges{j}(k,2);[~,truth]=ode113(@(~,z)sign(len)*[cos(z(3));sin(z(3));curvature],[0 abs(len)],q',odeset(RelTol=1e-12,AbsTol=1e-13));
   predicted=biagt.Advance(q,len,curvature);q=truth(end,:);odeError=max(odeError,max(abs([q(1:2)-predicted(1:2),atan2(sin(q(3)-predicted(3)),cos(q(3)-predicted(3)))])));
  end
 end
end
assert(endpoint<1e-9&&odeError<1e-8&&max(abs(L-reverse))<1e-9);
connection=reedsSheppConnection('MinTurningRadius',v.turning_radius_min,'ForwardCost',1,'ReverseCost',1);minimumError=0;
for j=1:30,[~,cost]=connect(connection,Q(j,:),P(j,:));minimumError=max(minimumError,abs(L(j)-cost(1)));end
assert(minimumError<1e-8);
body=[v.lw+v.lf,v.lr,v.lb/2];thin={[4.9,-.1;4.91,-.1;4.91,.1;4.9,.1]};assert(~biagt.Free([0,0,0],[6,0],c,o,body,thin));
assert(biagt.Free([0,0,0],[6,0],c,o,body,{[4.9,5;4.91,5;4.91,5.1;4.9,5.1]}));
% The native article ends in a ball. Test the exact-pose adapter separately.
[arcs,info]=biagt.Search(c,o);assert(~isempty(arcs));hError=0;density=inf;branchError=0;modeUses=[0 0];
for side=1:2
 A=info.trees{side};B=info.trees{3-side};modeUses=modeUses+sum(A.tried,1);
 assert(all(A.priority(:)==0|A.priority(:)==1));
 for j=2:A.n
  parent=A.parent(j);q=biagt.Advance(A.q(parent,:),A.arc(j,1),A.arc(j,2));branchError=max(branchError,max(abs([q(1:2)-A.q(j,1:2),atan2(sin(q(3)-A.q(j,3)),cos(q(3)-A.q(j,3)))])));
  assert(parent<j&&abs(A.g(j)-A.g(parent)-abs(A.arc(j,1)))<1e-10);
  density=min(density,min(biagt.Distance(A.q(j,:),A.q(1:j-1,:),o.metric)));small=B;small.n=A.otherCount(j);
  h=biagt.Heuristic(A.q(j,:),A.target,small,c,o);hError=max(hError,abs(h-A.h(j)));
 end
end
assert(branchError<1e-10&&hError<1e-10&&density>=o.delta-1e-10&&all(modeUses>0));
assert(laumond.ArcsFree([c.task.x0,c.task.y0,c.task.theta0],arcs,c));
report=struct('passed',true,'random_connections',300,'ODE_connections',30,'shortest_length_comparisons',30,'endpoint_error',endpoint,'ODE_error',odeError,'shortest_length_error',minimumError,'branch_error',branchError,'heuristic_error',hError,'minimum_node_distance',density,'mode_expansions',modeUses,'thin_obstacle_rejected',true);disp(report);
end
