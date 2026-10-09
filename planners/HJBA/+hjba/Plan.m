function result=Plan(c)
o=hjba.Config();result=cp.EmptyResult('HJBA',c.id,'path');previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous));old=rng;rng(o.seedBase+c.id);restoreRng=onCleanup(@()rng(old)); %#ok<NASGU>
folder=getenv('COMMONPARKING_HJBA_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','hjba',computer('arch'));end;if isfolder(folder),addpath(folder);end
clock=tic;[samples,guidance]=hjba.Guidance(c,o);preparation=toc(clock);result.diagnostics=struct('options',o,'guidance',guidance);
if isempty(samples),result.status.code='empty_guidance';result.status.message='No collision-free numerical BRT node satisfies the paper global-coordinate sampling quadrant.';return;end
count=size(samples,1);branches=cell(count,1);pool=gcp('nocreate');owned=isempty(pool);startupAttempts=0;
if owned
 cluster=parcluster('Processes');
 for attempt=1:2
  startupAttempts=attempt;
  try,pool=parpool(cluster,min([count,o.maximumWorkers,cluster.NumWorkers]));break;
  catch problem,if attempt==2,rethrow(problem);end,end
 end
 cleanup=onCleanup(@()delete(pool)); %#ok<NASGU>
end
workers=min(o.maximumWorkers,pool.NumWorkers);
parfor (k=1:count,workers),branches{k}=hjba.Branch(c,samples(k,:),guidance.bounds,o);end
costs=cellfun(@(a)a.cost,branches);[cost,best]=min(costs);result.solver=struct('success',isfinite(cost),'connected_states',count,'successful_branches',nnz(isfinite(costs)),'branch_costs',costs,'workers',workers,'pool_startup_attempts',startupAttempts,'guidance_time_s',preparation,'best_branch',best);
result.diagnostics.branches=branches;
if ~isfinite(cost),result.status.code='search_budget_exhausted';result.status.message='All connected-state branches failed within their disclosed finite search budgets.';return;end
p=branches{best}.primitives;start=[c.task.x0 c.task.y0 c.task.theta0];result.path=cp.PathFromArcs(start,p,c.vehicle,o.outputSpacing);result.diagnostics.primitives=p;
endpoint=[result.path.x(end)-c.task.xf,result.path.y(end)-c.task.yf,atan2(sin(result.path.theta(end)-c.task.thetaf),cos(result.path.theta(end)-c.task.thetaf))];
assert(max(abs(endpoint))<1e-6&&laumond.ArcsFree(start,p,c));
result.status=struct('success',true,'code','path_found','message','Numerical HJ guidance and connected-state bidirectional search returned the shortest successful branch; continuous full-body arc checks passed.');
end
