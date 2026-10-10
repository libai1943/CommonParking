function result=Plan(c)
o=teb.Config();o.vehicle=c.vehicle;env=teb.Prepare(c);result=cp.EmptyResult('TEB',c.id,'trajectory');
state=rng;previous=maxNumCompThreads(1);cleanupRng=onCleanup(@()rng(state));cleanupThreads=onCleanup(@()maxNumCompThreads(previous));rng(o.seedBase+c.id); %#ok<NASGU>
raw=parking.SearchHybridAStar(c,o.search);result.diagnostics=struct('options',o,'search',raw,'cycles',{{}});
candidates=struct('pose',{},'dt',{},'cost',{},'signature',{},'origin',{},'solver',{});
if raw.success
 p=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],raw.primitives,c.vehicle,o.spacing);
 b=teb.Initialize([p.x p.y p.theta],o,'global_hybrid_astar');b.signature=teb.Signature(b.pose(:,1:2),env);candidates=b;
end
timer=tic;
for cycle=1:o.planningCycles
 candidates=teb.Maintain(candidates,c,env,o);[candidates,exploration]=teb.Explore(candidates,c,env,o);
 entry=struct('exploration',exploration,'stages',{{}});keep=true(size(candidates));
 for k=1:numel(candidates)
  b=candidates(k);
  for resize=1:o.resizeCalls
   if toc(timer)>o.maxSeconds,result.status.code='optimization_time_limit';result.diagnostics.candidates=candidates;return;end
   b=teb.Resize(b,o);[b,info]=teb.Optimize(b,env,o);entry.stages{k,resize}=info;
   if ~info.success,keep(k)=false;break;end
  end
  candidates(k)=b;
 end
 candidates=candidates(keep);result.diagnostics.cycles{cycle}=entry;
end
if isempty(candidates),result.status.code='no_optimized_candidate';result.status.message='Topology exploration/LM produced no valid finite candidate.';return;end
[~,index]=min([candidates.cost]);best=candidates(index);[q,export]=teb.Export(best,env,o);
result.diagnostics.candidates=candidates;result.diagnostics.selected=index;result.diagnostics.export=export;
[~,parts]=teb.Residual(teb.Pack(best),best,env,o);result.diagnostics.penalties=parts;
result.solver=struct('success',export.success,'code','fixed_budget_lm','selected_cost',best.cost, ...
 'cycles',o.planningCycles,'candidates',numel(candidates),'selected_origin',best.origin,'convergence_certificate',false);
if ~export.success,result.status.code=export.code;return;end
result.trajectory=q;result.status=struct('success',true,'code','finite_budget_completed', ...
 'message','Topology exploration and the fixed-budget soft-constraint LM completed; feasibility is assessed separately.');
end
