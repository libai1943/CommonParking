function [profile,info] = Speed(length,v,o)
% Constant-jerk temporal discretization of the cited ITSC 2019 formulation.
% TDR's modified QP omits the nonlinear centripetal term. Exact phase rest
% and the unpublished weights below are explicit benchmark choices.
T = o.horizonFactor*(v.vmax^2+length*v.amax)/(v.amax*v.vmax);
n = max(4,ceil(T/o.speedStep)+1);h = T/(n-1);
D = spdiags([-ones(n-1,1),ones(n-1,1)],0:1,n-1,n)/h;
w = o.speedWeights;
H = 2*blkdiag(sparse(n,n),w(3)*speye(n),w(1)*speye(n)+w(2)*(D'*D));
f = [zeros(n,1);-2*w(3)*v.vmax*ones(n,1);zeros(n,1)];
Aeq = sparse(2*(n-1),3*n);
for i=1:n-1
    Aeq(i,[i,i+1,n+i,2*n+i,2*n+i+1]) = [-1,1,-h,-h*h/3,-h*h/6];
    Aeq(n-1+i,[n+i,n+i+1,2*n+i,2*n+i+1]) = [-1,1,-h/2,-h/2];
end
lb = [zeros(2*n,1);-v.amax*ones(n,1)];
ub = [length*ones(n,1);v.vmax*ones(n,1);v.amax*ones(n,1)];
fixed = [1,n,n+1,2*n,2*n+1,3*n];value = [0,length,0,0,0,0];
lb(fixed)=value;ub(fixed)=value;
A = [sparse(n-1,2*n),D;sparse(n-1,2*n),-D];b = o.maxJerk*ones(2*(n-1),1);
[z,cost,flag,output] = quadprog(H,f,A,b,Aeq,zeros(2*(n-1),1),lb,ub,[],o.qp);
info = struct('success',false,'exitflag',flag,'message',output.message,'objective',cost,'horizon',T);
profile = [];
if flag<=0||isempty(z),return;end
info.residual = max([0;abs(Aeq*z);A*z-b;lb-z;z-ub]);
info.success = info.residual<o.feasibilityTolerance;
profile = struct('t',(0:n-1)'*h,'s',z(1:n),'v',z(n+1:2*n), ...
    'a',z(2*n+1:end),'jerk',diff(z(2*n+1:end))/h);
end
