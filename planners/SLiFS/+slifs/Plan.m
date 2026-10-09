function result=Plan(c)
clock=tic;options=slifs.Config();result=cp.EmptyResult('SLiFS',c.id,'trajectory');N=options.nodes;
previousThreads=maxNumCompThreads(1);restoreThreads=onCleanup(@()maxNumCompThreads(previousThreads)); %#ok<NASGU>
task=c.task;offset=c.vehicle.length-c.vehicle.length/(2*options.circles)-c.vehicle.lr;
ends=[task.x0 task.y0 task.theta0 0 0;task.xf task.yf task.thetaf 0 0];
ends=[ends,ends(:,1)+offset*cos(ends(:,3)),ends(:,2)+offset*sin(ends(:,3))];endpointOptions=options;endpointOptions.nodes=2;
[~,~,endpointGap]=slifs.FeasibleSet([ends(:);1],c,endpointOptions);
if endpointGap<0,result.status.code='endpoint_circle_cover_blocked';result.status.message=sprintf('The %d-circle vehicle cover with %.3g m clearance blocks a task endpoint.',options.circles,options.clearance);result.solver.endpoint_clearance_excess=endpointGap;return;end
[z,initialization]=slifs.Initialize(c,options);result.solver.search_success=initialization.success;
if ~initialization.success,result.status.code='initialization_failed';result.status.message='Hybrid A* initialization failed.';return;end
qp=slifs.Prepare(z,c,options);history=cell(0,1);total=0;outer=0;converged=false;nativeSuccess=false;
best=z;bestMerit=merit(z);bestFeasible=[];bestCost=inf;epsilon2=options.epsilon2;last=struct();
while total<options.maxIterations&&toc(clock)<options.maxSeconds
 outer=outer+1;anchor=z;[collisionA,collisionB]=slifs.FeasibleSet(anchor,c,options);radius=options.initialRadius;stagnant=0;
 for inner=1:options.maxInnerIterations
  if total>=options.maxIterations||toc(clock)>=options.maxSeconds,break;end
  total=total+1;old=z;[trial,last]=slifs.Subproblem(z,radius,qp,collisionA,collisionB,c,options);
  if ~last.success,break;end
  nativeSuccess=true;decrease=merit(old)-merit(trial);
  if inner==1
   z=trial;radius=radius/options.alpha; % Paper's deliberate first-step perturbation.
  elseif decrease < -options.epsilon1
   radius=radius/options.alpha;direction=trial-old;step=1;
   for ls=1:20
    candidate=old+step*direction;
    if merit(candidate)<merit(old),z=candidate;break;end
    step=step/2;
   end
  elseif decrease > options.epsilon1
   z=trial;nonlinear=sum(abs(slifs.Residual(z,N,c.vehicle,options.circles)));
   rho=max(last.linearized_penalty,epsilon2)/max(nonlinear,realmin);
   if rho<options.rho1,radius=radius/options.alpha;elseif rho>options.rho2,radius=radius*options.beta;end
   if nonlinear<options.epsilon1,radius=min(radius,options.smallRadius);end
   epsilon2=max(realmin,epsilon2/10);
  end
  G=slifs.Residual(z,N,c.vehicle,options.circles);cost=slifs.Objective(z,N);value=cost+options.penalty*sum(abs(G));
  if cost>=slifs.Objective(old,N)-options.epsilon1,stagnant=stagnant+1;else,stagnant=0;end
  if value<bestMerit,best=z;bestMerit=value;end
  [~,~,gap]=slifs.FeasibleSet(z,c,options);
  history{end+1}=struct('outer',outer,'inner',inner,'cost',cost,'penalty_l1',sum(abs(G)),'maximum_residual',max(abs(G)), ...
   'radius',radius,'exitflag',last.exitflag,'qp_iterations',last.iterations,'minimum_circle_clearance_excess',gap); %#ok<AGROW>
  if inner==1,continue;end
  if abs(decrease)<options.epsilon1||norm(z-anchor)<options.epsilon3||radius<options.minimumRadius||(sum(abs(G))<options.epsilon1&&stagnant>=3),break;end
 end
 if ~isempty(last)&&isfield(last,'success')&&~last.success,break;end
 % Compare the candidates at the ends of inner loops, as in Section III-D.
 G=slifs.Residual(z,N,c.vehicle,options.circles);cost=slifs.Objective(z,N);[~,~,gap]=slifs.FeasibleSet(z,c,options);
 if max(abs(G))<options.epsilon1&&gap>=-1e-6&&max([0;qp.A*z-qp.b;qp.lower-z;z-qp.upper])<1e-6&&cost<bestCost
  bestFeasible=z;bestCost=cost;
 end
 oldCost=slifs.Objective(anchor,N);newCost=slifs.Objective(z,N);penalty=sum(abs(slifs.Residual(z,N,c.vehicle,options.circles)));
 if ~(merit(z)<merit(anchor)||(penalty<options.epsilon1&&newCost<oldCost)),z=anchor;end
 if norm(z-anchor)<N*options.epsilon1,converged=true;break;end
end
if ~isempty(bestFeasible),chosen=bestFeasible;else,chosen=best;end
q=reshape(chosen(1:7*N),N,7);dt=chosen(end)/(N-1);a=[diff(q(:,4))/dt;0];omega=[diff(q(:,5))/dt;0];
result.trajectory=struct('t',linspace(0,chosen(end),N)','x',q(:,1),'y',q(:,2),'theta',q(:,3),'v',q(:,4),'phi',q(:,5),'a',a,'omega',omega);
% Appendix B's between-waypoint check, using swept corner segments and a
% fine interpolated footprint check to also resolve rotation between poses.
collision=slifs.SweptCollision(q(:,1:3),c);withinBudget=total<options.maxIterations&&toc(clock)<options.maxSeconds;
result.solver=struct('success',nativeSuccess&&converged&&withinBudget&&last.success&&~isempty(bestFeasible)&&~collision,'converged',converged,'iterations',total,'outer_iterations',outer, ...
 'maximum_residual',max(abs(slifs.Residual(chosen,N,c.vehicle,options.circles))),'objective',slifs.Objective(chosen,N),'continuous_reference_collision',collision,'last_subproblem',last);
result.diagnostics=struct('initialization',initialization,'history',{history},'options',options,'native_variables',chosen);
if result.solver.success
 result.status=struct('success',true,'code','solved','message','SLiFS converged within its budget, with successful native QP flags and a feasible collision-free candidate.');
else
 result.status.code='optimization_failed';result.status.message='SLiFS did not return a feasible, collision-free candidate within its resource bounds.';
end
 function value=merit(candidate)
  value=slifs.Objective(candidate,N)+options.penalty*sum(abs(slifs.Residual(candidate,N,c.vehicle,options.circles)));
 end
end
