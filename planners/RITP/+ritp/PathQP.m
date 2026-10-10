function [coeff,solver,problem]=PathQP(reference,gear,e,o)
S=[0;cumsum(hypot(diff(reference(:,1)),diff(reference(:,2))))];L=S(end);
M=max(o.minimumSamples,round(o.samplesPerMetre*L));s=linspace(0,L,M)';
B=ritp.Basis(S,L,o.degree,0);D=ritp.Basis(s,L,o.degree,1);DD=ritp.Basis(s,L,o.degree,2);
H0=2*(o.pathWeights(1)*(B'*(e.*B))+o.pathWeights(2)*(D'*D)+o.pathWeights(3)*(DD'*DD));
f0=-2*o.pathWeights(1)*(B'*(e.*reference(:,1:2)));n=o.degree+1;
ends=ritp.Basis([0;L],L,o.degree,0);tangent=ritp.Basis([0;L],L,o.degree,1);
heading=reference([1 end],3);
% Exact physical unit tangent at both endpoints, including reverse motion.
Aeq=blkdiag([ends;tangent],[ends;tangent]);
beq=[reference([1 end],1);gear*cos(heading);reference([1 end],2);gear*sin(heading)];
A=zeros(0,2*n);b=zeros(0,1);
H=blkdiag(H0,H0);f=f0(:);H=(H+H')/2;
options=optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'ConstraintTolerance',1e-10,'MaxIterations',500);
[z,objective,flag,output]=quadprog(H,f,A,b,Aeq,beq,[],[],[],options);
solver=struct('success',false,'exitflag',flag,'output',output,'objective_without_constant',objective);
coeff=[];if ~isempty(z)&&isreal(z)&&all(isfinite(z))
 coeff=reshape(z,n,2);solver.equality_residual=max(abs(Aeq*z-beq));
 solver.inequality_residual=max([0;A*z-b]);solver.success=flag>0&&max(solver.equality_residual,solver.inequality_residual)<=o.feasibilityTolerance;
end
problem=struct('H',H,'f',f,'A',A,'b',b,'Aeq',Aeq,'beq',beq,'S',S,'L',L,'s',s,'weights',e);
end
