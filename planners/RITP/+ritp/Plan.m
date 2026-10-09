function result=Plan(c)
o=ritp.Config();result=cp.EmptyResult('RITP',c.id,'trajectory');
previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
result.diagnostics=struct('options',o);raw=parking.SearchHybridAStar(c,o.search);result.diagnostics.search=raw;
if ~raw.success,result.status.code='initialization_failed';result.status.message='Hybrid A* did not find the reference path.';return;end
t=c.task;p=cp.PathFromArcs([t.x0 t.y0 t.theta0],raw.primitives,c.vehicle,o.referenceSpacing);
result.diagnostics.initial_path=p;cuts=p.cusp_indices;count=numel(cuts)-1;phases=cell(count,1);q=[];offset=0;
references=cell(count,1);gears=zeros(count,1);
for k=1:count,rows=cuts(k):cuts(k+1);references{k}=[p.x(rows) p.y(rows) p.theta(rows)];gears(k)=p.gear(rows(1));end
pool=gcp('nocreate');owned=isempty(pool);
if owned,pool=parpool('Processes',min(o.maximumWorkers,count));cleanup=onCleanup(@()delete(pool));end %#ok<NASGU>
workers=min(o.maximumWorkers,pool.NumWorkers);result.diagnostics.parallel=struct('workers',workers,'owned_pool',owned);
parfor (k=1:count,workers)
 phases{k}=ritp.Phase(references{k},gears(k),c,o);
end
result.diagnostics.phases=phases;
for k=1:count
 phase=phases{k};
 if ~phase.success,result.solver=phase.solver;result.solver.success=false;result.status.code=phase.code;result.status.message='A gear phase did not satisfy the paper path/velocity QP and sampled collision termination conditions.';return;end
 local=phase.trajectory;
 fields=fieldnames(local);keep=(1:numel(local.t))';if k>1,keep=keep(2:end);end
 if isempty(q),q=local;else,for j=1:numel(fields),f=fields{j};a=local.(f);if strcmp(f,'t'),a=a+offset;end;q.(f)=[q.(f);a(keep)];end,end
 offset=offset+phase.velocity.T;
end
result.trajectory=q;result.solver=struct('success',true,'phase_count',numel(phases),'path_iterations',cellfun(@(a)a.iterations,phases));
result.status=struct('success',true,'code','solved','message','All paper path and velocity QPs completed; each path phase passed the native sampled mutual-vertex test.');
end
