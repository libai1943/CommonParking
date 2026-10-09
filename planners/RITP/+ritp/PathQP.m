function [coeff,solver,problem]=PathQP(reference,gear,e,o)
S=[0;cumsum(hypot(diff(reference(:,1)),diff(reference(:,2))))];L=S(end);
M=max(o.minimumSamples,round(o.samplesPerMetre*L));s=linspace(0,L,M)';d=L/(M-1);
B=ritp.Basis(S,L,o.degree,0);D=ritp.Basis(s,L,o.degree,1);DD=ritp.Basis(s,L,o.degree,2);
H0=2*(o.pathWeights(1)*(B'*(e.*B))+o.pathWeights(2)*(D'*D)+o.pathWeights(3)*(DD'*DD));
f0=-2*o.pathWeights(1)*(B'*(e.*reference(:,1:2)));n=o.degree+1;
ends=ritp.Basis([0;L],L,o.degree,0);align=ritp.Basis([o.smoothingOrder*d;L-o.smoothingOrder*d],L,o.degree,0);
Aeq=[ends zeros(2,n);zeros(2,n) ends;align zeros(2,n)];
beq=[reference([1 end],1);reference([1 end],2); ...
 reference(1,1)+gear*o.smoothingOrder*d*cos(reference(1,3)); ...
 reference(end,1)-gear*o.smoothingOrder*d*cos(reference(end,3))];
H=blkdiag(H0,H0);f=f0(:);H=(H+H')/2;
options=optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'ConstraintTolerance',1e-10,'MaxIterations',500);
[z,objective,flag,output]=quadprog(H,f,[],[],Aeq,beq,[],[],[],options);
solver=struct('success',false,'exitflag',flag,'output',output,'objective_without_constant',objective);
coeff=[];if ~isempty(z)&&isreal(z)&&all(isfinite(z))
 coeff=reshape(z,n,2);solver.equality_residual=max(abs(Aeq*z-beq));
 solver.success=flag>0&&solver.equality_residual<=o.feasibilityTolerance;
end
problem=struct('H',H,'f',f,'Aeq',Aeq,'beq',beq,'S',S,'L',L,'s',s,'weights',e);
end
