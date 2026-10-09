function [z,ctx,records,success]=Regularize(z,ctx)
% Adapt the logarithmic barrier continuation step without changing its target.
ctx.mode='obstacle';target=ctx.options.barriers(end);mu=ctx.mu;step=log(1/.7);records={};success=false;started=tic;
for attempt=1:120
 trial=max(target,mu*exp(-step));ctx.mu=trial;[candidate,info]=indpark.Solve(z,ctx);info.barrier=trial;records{end+1}=info;
 fprintf('Barrier continuation %.8g success%d residual %.3g\n',trial,info.success,info.residual);
 if info.success
  z=candidate;mu=trial;step=min(log(1/.7),1.3*step);if mu==target,success=true;break;end
 else
  step=step/2;if step<1e-3,break;end
 end
 if toc(started)>180,break;end
end
ctx.mu=mu;
end
