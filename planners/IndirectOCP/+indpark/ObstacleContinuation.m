function [z,ctx,records,success]=ObstacleContinuation(z,ctx)
% Continue the running cost from the solved tracking OCP to Eq. (11).
ctx.mode='blend';beta=0;step=.02;records={};started=tic;success=false;
for attempt=1:120
 trial=min(1,beta+step);ctx.blend=trial;[candidate,info]=indpark.Solve(z,ctx);info.blend=trial;records{end+1}=info;
 fprintf('Obstacle continuation %.7f success%d residual %.3g\n',trial,info.success,info.residual);
 if info.success
  z=candidate;beta=trial;step=min(.2,1.5*step);if beta==1,success=true;break;end
 else
  step=step/2;if step<1e-5,break;end
 end
 if toc(started)>180,break;end
end
ctx.blend=beta;
end
