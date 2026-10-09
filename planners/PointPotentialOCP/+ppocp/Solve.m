function [z,info]=Solve(c,N,points,z,o)
if nargin<5,o=ppocp.Config();end
if nargin<4||isempty(z),z=ppocp.Guess(c,N,o,false);end
ctx=struct('grid',linspace(0,1,N),'points',points,'vehicle',c.vehicle);t=c.task;
lower=repmat([-Inf;-Inf;-Inf;-c.vehicle.vmax;-c.vehicle.phimax],N,1);upper=-lower;
start=[t.x0;t.y0;t.theta0;0;0];goal=[t.xf;t.yf;t.thetaf;0];
lower(1:5)=start;upper(1:5)=start;ix=(N-1)*5+(1:4);lower(ix)=goal;upper(ix)=goal;
lower=[lower;.05];upper=[upper;600];started=tic;
options=optimoptions('fmincon',Algorithm='sqp',SpecifyObjectiveGradient=true, ...
    SpecifyConstraintGradient=true,ConstraintTolerance=o.constraintTolerance, ...
    OptimalityTolerance=o.optimalityTolerance,StepTolerance=o.stepTolerance, ...
    MaxIterations=o.maxIterations,MaxFunctionEvaluations=o.maxEvaluations, ...
    Display='off',OutputFcn=@stop);
[z,cost,flag,out]=fmincon(@objective,z,[],[],[],[],lower,upper,@(x)ppocp.Constraints(x,ctx),options);
[ineq,eq]=ppocp.Constraints(z,ctx);violation=max([0;ineq;abs(eq);lower-z;z-upper]);
info=struct('success',flag>0&&violation<=o.constraintTolerance,'flag',flag,'cost',cost, ...
    'output',out,'seconds',toc(started),'infeasibility',violation,'nodes',N);
fprintf('PointPotential SQP N%d points%d flag%d residual%.3g T%.5f\n',N,size(points,1),flag,violation,z(end));
function [f,g]=objective(z),f=z(end);g=zeros(size(z));g(end)=1;end
function value=stop(~,~,~),value=toc(started)>o.maxSeconds;end
end
