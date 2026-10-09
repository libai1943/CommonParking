function report=TestTDR()
o=tdr.Config();c=LoadCase(3);v=c.vehicle;profileResidual=0;odeError=0;
for L=[.1,1,5,20]
 [q,info]=tdr.Speed(L,v,o);assert(info.success);profileResidual=max(profileResidual,info.residual);
 for i=1:numel(q.t)-1
  h=q.t(i+1)-q.t(i);z=[q.s(i);q.v(i);q.a(i)];
  [~,states]=ode45(@(~,z)[z(2);z(3);q.jerk(i)],[0,h],z,odeset('RelTol',1e-11,'AbsTol',1e-12));
  odeError=max(odeError,max(abs(states(end,:)-[q.s(i+1),q.v(i+1),q.a(i+1)])));
 end
end
p=parking.PolygonData(c);poses=[c.task.x0,c.task.y0,c.task.theta0;c.task.xf,c.task.yf,c.task.thetaf];
[lambda,mu,d,info]=tdr.DualSeed(c,poses,p,o);last=cumsum(cellfun(@(z)size(z,1),p.A));first=last-cellfun(@(z)size(z,1),p.A)+1;
G=[1 0;-1 0;0 1;0 -1];g=[v.lw+v.lf;v.lr;v.lb/2;v.lb/2];qpError=0;dualError=0;count=0;
for i=1:2
 R=[cos(poses(i,3)),-sin(poses(i,3));sin(poses(i,3)),cos(poses(i,3))];
 for j=1:p.count
  A=p.A{j};b=p.b{j};n=size(A,1);coef=[A*poses(i,1:2)'-b;-g];
  H=2/o.dualBeta*blkdiag(A*A',zeros(4));
  [z,cost,flag]=quadprog(H,-coef,-coef',0,[R'*A',G'],zeros(2,1),zeros(n+4,1),[],[],o.qp);
  assert(flag>0);qpError=max(qpError,abs(cost-info.unconstrained_QP_objective(i,j)));
  l=lambda(i,first(j):last(j))';m=reshape(mu(i,j,:),4,1);normal=A'*l;
  dualError=max([dualError;abs(G'*m+R'*normal);abs(coef'*[l;m]+d(i,j));normal'*normal-1]);count=count+1;
 end
end
assert(qpError<1e-6&&dualError<1e-8&&odeError<1e-8);
report=struct('passed',true,'speed_QPs',4,'profile_residual',profileResidual,'speed_ODE_error',odeError,'dual_QPs',count,'dual_objective_error',qpError,'dual_feasibility_error',dualError);disp(report);
end
