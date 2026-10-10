function [P,info]=Inner(reference,bubble,kappaMax,o,timer)
% Algorithm 1: penalty, successive linearization, and trust-region loops.
n=size(reference,1);P=reference;scale=(mean(vecnorm(diff(reference),2,2)))^4;
D=spdiags([ones(n-2,1),-2*ones(n-2,1),ones(n-2,1)],0:2,n-2,n);
Q=kron(speye(2),D'*D)/scale;H=blkdiag(2*Q,sparse(n-2,n-2));
low=reference-bubble/sqrt(2);high=reference+bubble/sqrt(2);
fixed=[1 n];low(fixed,:)=reference(fixed,:);high(fixed,:)=reference(fixed,:);
first=reference(2,:)-reference(1,:);first=first/norm(first);
last=reference(end-1,:)-reference(end,:);last=last/norm(last);
normal=[-first(2),first(1);-last(2),last(1)];
Aeq=sparse([1 1 2 2],[2 n+2 n-1 2*n-1],[normal(1,:) normal(2,:)],2,3*n-2);
beq=[normal(1,:)*reference(1,:)';normal(2,:)*reference(end,:)'];
headingA=sparse([1 1 2 2],[2 n+2 n-1 2*n-1],[-first -last],2,3*n-2);
headingB=[-first*reference(1,:)';-last*reference(end,:)'];
mu=o.initialPenalty;radius=o.initialTrust;history=zeros(0,8);success=false;lastFlag=NaN;
reason='iteration_limit';total=0;
for penalty=1:o.maxPenaltyIterations
 subConverged=false;
 for iteration=1:o.maxSubIterations
  if toc(timer)>o.maxOptimizationSeconds,reason='time_limit';break;end
  z=P(:);[g,J]=dliaps.CurvatureConstraint(P,kappaMax,scale);oldCost=sum((D*P).^2,'all')/scale;
  oldMerit=oldCost+mu*sum(max(g,0));accepted=false;converged=false;
  while radius>=o.xTolerance
   A=[J,-speye(n-2);headingA];b=[J*z-g;headingB];
   lower=[max(low(:),z-radius);zeros(n-2,1)];upper=[min(high(:),z+radius);inf(n-2,1)];
   f=[zeros(2*n,1);mu*ones(n-2,1)];
   objectiveScale=max(1,mu);
   [trial,~,lastFlag,output]=quadprog(H/objectiveScale,f/objectiveScale,A,b,Aeq,beq,lower,upper,[],o.qp);total=total+1;
   if lastFlag<=0||isempty(trial),reason='qp_failed';break;end
   if max([0;A*trial-b;abs(Aeq*trial-beq);lower-trial;trial-upper])>1e-6,reason='qp_residual';lastFlag=-99;break;end
   znew=trial(1:2*n);step=znew-z;cost=sum((D*reshape(znew,n,2)).^2,'all')/scale;
   modelImprove=oldMerit-cost-mu*sum(max(g+J*step,0));
   next=reshape(znew,n,2);newG=dliaps.CurvatureConstraint(next,kappaMax,scale);
   trueImprove=oldMerit-cost-mu*sum(max(newG,0));
   ratio=trueImprove/max(modelImprove,realmin);
   % An already solved subproblem has no meaningful improvement ratio.
   converged=norm(step,Inf)<o.xTolerance||abs(modelImprove)<o.fTolerance;
   if converged
    if trueImprove>=-o.fTolerance,P=next;accepted=true;end
   elseif modelImprove>0&&ratio>o.rho
    P=next;accepted=true;radius=min(o.maximumTrust,radius*o.trustGrow);
   else
    radius=radius*o.trustShrink;
   end
   history(end+1,:)=[penalty,iteration,mu,radius,modelImprove,trueImprove,ratio,output.iterations]; %#ok<AGROW>
   if accepted||converged,break;end
   if toc(timer)>o.maxOptimizationSeconds,reason='time_limit';break;end
  end
  if lastFlag<=0||toc(timer)>o.maxOptimizationSeconds,break;end
  if converged||radius<o.xTolerance,subConverged=true;break;end
 end
 if lastFlag<=0||toc(timer)>o.maxOptimizationSeconds,break;end
 if ~subConverged,reason='subproblem_iteration_limit';end
 violation=max(dliaps.CurvatureConstraint(P,kappaMax,scale));
 if subConverged&&violation<=o.constraintTolerance,success=true;reason='converged';break;end
 mu=mu*o.penaltyFactor;radius=o.initialTrust;
end
info=struct('success',success,'code',reason,'exitflag',lastFlag,'qp_calls',total, ...
 'maximum_normalized_quartic_residual',max(dliaps.CurvatureConstraint(P,kappaMax,scale)), ...
 'constraint_scale',scale,'history',history,'bubble_radii',bubble,'reference',reference);
end
