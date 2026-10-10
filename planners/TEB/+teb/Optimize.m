function [b,info]=Optimize(b,env,o)
% Sparse LM: column scaling of the damped normal equations, fixed penalties.
z=teb.Pack(b);[pattern,colors]=teb.Pattern(size(b.pose,1),numel(env.polygons));
r=teb.Residual(z,b,env,o);cost=r'*r;initial=cost;mu=1e-3;accepted=0;rejected=0;code='iteration_budget';
for iteration=1:o.lmIterations
 J=teb.Jacobian(z,r,b,env,o,pattern,colors);g=J'*r;H=J'*J;
 if any(~isfinite(r))||any(~isfinite(nonzeros(J))),code='nonfinite_model';break;end
 if norm(g,Inf)<o.gradientTolerance,code='stationary';break;end
 diagonal=max(full(diag(H)),1e-6);scale=spdiags(1./sqrt(diagonal),0,numel(z),numel(z));Hs=scale*H*scale;gs=scale*g;taken=false;
 for attempt=1:o.lmTrials
  step=-scale*((Hs+mu*speye(numel(z)))\gs);trial=z+step;
  if norm(step)<=o.stepTolerance*(1+norm(z)),code='small_step';break;end
  dt=trial(5*(size(b.pose,1)-2)+1:end);
  if all(dt>o.minimumDt)&&all(isfinite(trial))
   next=teb.Residual(trial,b,env,o);nextCost=next'*next;prediction=-2*g'*step-step'*H*step;
   if isfinite(nextCost)&&nextCost<cost&&prediction>0
    ratio=(cost-nextCost)/prediction;z=trial;r=next;cost=nextCost;accepted=accepted+1;taken=true;
    mu=max(1e-12,mu*max(1/3,1-(2*ratio-1)^3));break;
   end
  end
  mu=min(1e16,mu*4);rejected=rejected+1;
 end
 if strcmp(code,'small_step'),break;end
 if ~taken,code='no_descent';break;end
end
[b.pose,b.dt]=teb.Unpack(z,b);b.cost=cost;
success=isfinite(cost)&&all(isfinite(z))&&all(b.dt>o.minimumDt)&&~strcmp(code,'nonfinite_model')&& ...
 (accepted>0||ismember(code,{'stationary','small_step'}));
info=struct('success',success,'code',code,'iterations',iteration,'accepted_steps',accepted,'rejected_steps',rejected, ...
 'initial_cost',initial,'cost',cost,'gradient_inf',norm(g,Inf));b.solver=info;
end
