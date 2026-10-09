function [z,info]=Solve(z,ctx)
started=tic;o=ctx.options;trace=[];success=false;reason='iteration_limit';N=o.nodes;
scale=repmat([1;1;1;1;1;10;10;10;10;10],N,1);scale=[scale;1];
for iteration=1:o.maxIterations
 [r,J]=indpark.Residual(z,ctx);res=norm(r,inf);
 if res<o.tolerance,success=true;reason='converged';break;end
 if toc(started)>o.maxSeconds,reason='time_limit';break;end
 L=decomposition(J,'lu');step=-(L\r);base=norm(step./scale);alpha=1;accepted=false;
 if any(~isfinite(step)),reason='singular_jacobian';break;end
 for bt=1:o.maxBacktracks
  candidate=z+alpha*step;Y=reshape(candidate(1:end-1),10,N);
  mid=(Y(:,1:end-1)+Y(:,2:end))/2;
  feasible=all(abs(mid(4,:))<ctx.vehicle.vmax)&&all(abs(mid(5,:))<ctx.vehicle.phimax)&&all(abs(Y(5,:))<pi/2-1e-4)&&candidate(end)>log(.05)&&candidate(end)<log(600);
  if feasible
   next=indpark.Residual(candidate,ctx);correct=L\next;
   if all(isfinite(next))&&norm(correct./scale)<=(1-alpha/2)*base,accepted=true;break;end
  end
  alpha=alpha/2;
 end
 trace=[trace;iteration,res,alpha,toc(started)]; %#ok<AGROW>
 if ~accepted,reason='line_search_failed';break;end
 z=candidate;if mod(iteration,10)==0,fprintf('Indirect %s mu=%g iter%d residual %.3g T%.3f\n',ctx.mode,ctx.mu,iteration,res,exp(z(end)));end
end
res=norm(indpark.Residual(z,ctx),inf);if res<o.tolerance,success=true;reason='converged';end
info=struct('success',success,'code',reason,'iterations',iteration,'residual',res,'seconds',toc(started),'trace',trace);
end
