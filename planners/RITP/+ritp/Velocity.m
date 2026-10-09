function speed=Velocity(L,numberOfPathSamples,v,o)
% Equations (26)--(35), quintic time law with all six endpoint equalities.
if L<v.vmax^2/v.amax,Tmin=2*sqrt(L/v.amax);else,Tmin=v.vmax/v.amax+L/v.vmax;end
T=o.timeRatio*Tmin;M=max(o.minimumSamples,floor(numberOfPathSamples*o.velocitySampleRatio));t=linspace(0,T,M)';
B=ritp.Basis(t,T,5,0);D=ritp.Basis(t,T,5,1);DD=ritp.Basis(t,T,5,2);DDD=ritp.Basis(t,T,5,3);inside=2:M-1;
H=2*(o.velocityWeights(1)*(D(inside,:)'*D(inside,:))+o.velocityWeights(2)*(DD(inside,:)'*DD(inside,:))+o.velocityWeights(3)*(DDD(inside,:)'*DDD(inside,:)));
Aeq=[B([1 end],:);D([1 end],:);DD([1 end],:)];beq=[0;L;0;0;0;0];
A=[D(inside,:);-D(inside,:);DD(inside,:);-DD(inside,:)];b=[v.vmax*ones(2*numel(inside),1);v.amax*ones(2*numel(inside),1)];
options=optimoptions('quadprog','Display','off','OptimalityTolerance',1e-10,'ConstraintTolerance',1e-10,'MaxIterations',500);
[coeff,objective,flag,output]=quadprog((H+H')/2,zeros(6,1),A,b,Aeq,beq,[],[],[],options);
speed=struct('success',false,'exitflag',flag,'output',output,'coefficients',coeff,'T',T,'t',t,'objective',objective);
if isempty(coeff)||~isreal(coeff)||any(~isfinite(coeff)),return;end
speed.s=B*coeff;speed.v=D*coeff;speed.a=DD*coeff;
speed.equality_residual=max(abs(Aeq*coeff-beq));speed.inequality_residual=max([0;A*coeff-b]);
speed.success=flag>0&&max(speed.equality_residual,speed.inequality_residual)<=o.feasibilityTolerance;
speed.problem=struct('H',H,'A',A,'b',b,'Aeq',Aeq,'beq',beq);
end
