function [trial,info]=Subproblem(z,radius,qp,collisionA,collisionB,c,options)
% Problem 2: convex QP with L1 epigraph variables and infinity trust region.
N=options.nodes;[G,A]=slifs.Residual(z,N,c.vehicle,options.circles);D=numel(z);M=numel(G);
rhs=A*z-G;constraints=[qp.A sparse(size(qp.A,1),M);collisionA sparse(size(collisionA,1),M);A -speye(M);-A -speye(M)];
b=[qp.b;collisionB;rhs;-rhs];
H=blkdiag(qp.H/options.penalty,sparse(M,M));f=[qp.f/options.penalty;ones(M,1)];
lower=[max(qp.lower,z-radius);zeros(M,1)];upper=[min(qp.upper,z+radius);inf(M,1)];
settings=optimoptions('quadprog','Algorithm','interior-point-convex','Display','off', ...
 'MaxIterations',options.qpMaxIterations,'ConstraintTolerance',options.qpTolerance,'OptimalityTolerance',options.qpTolerance,'StepTolerance',1e-12);
[solution,~,flag,output]=quadprog(H,f,constraints,b,[],[],lower,upper,[],settings);
info=struct('success',false,'exitflag',flag,'iterations',output.iterations,'message',output.message,'linear_violation',inf,'linearized_penalty',inf);trial=[];
if numel(solution)~=D+M||~isreal(solution)||any(~isfinite(solution)),return;end
trial=solution(1:D);info.linear_violation=max([0;constraints*solution-b;lower-solution;solution-upper]);
info.linearized_penalty=sum(abs(G+A*(trial-z)));
info.success=flag>0&&info.linear_violation<1e-6;
end
