function [x,info]=ConjugateGradient(fun,x,opt)
% Handwritten nonlinear Polak-Ribiere+ CG with Armijo backtracking and restarts.
% No fminunc/fmincon/third-party CG is used. Paper does not prescribe a CG variant.
[f,g]=fun(x);direction=-g;history=zeros(opt.maxIterations+1,3);history(1,:)=[f,norm(g,inf),0];
flag=0;message='iteration_limit';iter=0;
for iter=1:opt.maxIterations
    if norm(g,inf)<opt.gradientTolerance,flag=1;message='gradient_tolerance';break;end
    if direction'*g>=-1e-12*norm(g)*max(norm(direction),1),direction=-g;end
    slope=g'*direction;step=min(1,0.3/max(norm(direction,inf),1e-12));accepted=false;
    for back=1:35
        candidate=x+step*direction;[fn,gn]=fun(candidate);
        if isfinite(fn)&&all(isfinite(gn))&&fn<=f+1e-4*step*slope,accepted=true;break;end
        step=step/2;
    end
    if ~accepted,flag=-2;message='line_search_stalled';break;end
    beta=max(0,gn'*(gn-g)/max(g'*g,1e-30));
    if mod(iter,25)==0||beta>10,beta=0;end
    change=norm(candidate-x,inf);x=candidate;f=fn;direction=-gn+beta*direction;g=gn;
    history(iter+1,:)=[f,norm(g,inf),step];
    if change<opt.stepTolerance,flag=2;message='step_tolerance';break;end
end
info=struct('exitflag',flag,'message',message,'iterations',iter,'objective',f, ...
    'gradient_norm',norm(g,inf),'history',history(1:iter+1,:));
end
