function result=Plan(c)
o=tpckc.Config();result=cp.EmptyResult('TPCKC',c.id,'trajectory');previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
raw=parking.SearchHybridAStar(c,o.search);result.diagnostics=struct('options',o,'search',raw);
if ~raw.success,result.status.code='initialization_failed';result.status.message='Hybrid A* did not connect the task.';return;end
t=c.task;path=cp.PathFromArcs([t.x0 t.y0 t.theta0],raw.primitives,c.vehicle,.05);[initial,details]=tpckc.Initialize(path,c,o);
goalHeading=t.thetaf+2*pi*round((initial.theta(end)-t.thetaf)/(2*pi));keys=zeros(0,4);seeds=cell(0,1);certificates=cell(0,1);history={};
[q,solver,native]=tpckc.Solve(c,initial,keys,seeds,o.trust(1),o,goalHeading);
history{1}=struct('stage','first','solver',solver,'native',native,'new_key_count',0);success=false;termination='first_problem_failed';iteration=0;
if solver.success
 termination='outer_iteration_limit';
 for iteration=1:o.maximumOuterIterations
  violations=tpckc.Collect(q,c,0);finalTrial=isempty(violations);newCount=0;
  if finalTrial
   reference=q;radius=o.trust(3);stage='final_trial';
  else
   propagated=tpckc.Collect(q,c,round(o.propagationTime/o.resamplingTime));new=setdiff(propagated,keys,'rows','stable');newCount=size(new,1);
   if isempty(new),termination='no_new_key_constraints';break;end
   if iteration==1,reference=initial;else,reference=q;end
   [newSeeds,newCertificates]=tpckc.DualSeed(new,reference,c);keys=[keys;new];seeds=[seeds;newSeeds];certificates=[certificates;newCertificates]; %#ok<AGROW>
   radius=o.trust(2);stage='intermediate';
  end
  [q,solver,native]=tpckc.Solve(c,reference,keys,seeds,radius,o,goalHeading);
  history{end+1}=struct('stage',stage,'solver',solver,'native',native,'new_key_count',newCount); %#ok<AGROW>
  if ~solver.success,termination=[stage '_failed'];break;end
  if finalTrial&&isempty(tpckc.Collect(q,c,0)),success=true;termination='final_trial_vertex_clear';break;end
 end
end
result.solver=solver;result.solver.success=success;result.solver.outer_iterations=iteration;result.solver.key_constraints=size(keys,1);result.solver.termination=termination;
if ~isempty(q),result.trajectory=q;end
result.diagnostics.initial=initial;result.diagnostics.initialization=details;result.diagnostics.history=history;result.diagnostics.native=native;result.diagnostics.dual_certificates=certificates;
if success,result.status=struct('success',true,'code','solved','message','The final-trial implicit-Euler NLP and native vertex-intrusion checks passed. Full-body execution is evaluated independently.');
else,result.status.code=termination;result.status.message='TPCKC did not finish a successful, native vertex-clear final trial; no fallback trajectory was substituted.';end
end
