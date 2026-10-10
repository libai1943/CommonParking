function result=Plan(c)
o=ritp.Config();result=cp.EmptyResult('RITP',c.id,'trajectory');
previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
result.diagnostics=struct('options',o);seedCase=c;
seedCase.vehicle.kappa_max=o.searchCurvatureFraction*c.vehicle.kappa_max;
seedCase.vehicle.turning_radius_min=1/seedCase.vehicle.kappa_max;
seedCase.vehicle.phimax=atan(seedCase.vehicle.kappa_max*c.vehicle.lw);
raw=parking.SearchHybridAStar(seedCase,o.search);result.diagnostics.search=raw;
if ~raw.success,result.status.code='initialization_failed';result.status.message='Hybrid A* did not find the reference path.';return;end
t=c.task;p=cp.PathFromArcs([t.x0 t.y0 t.theta0],raw.primitives,c.vehicle,o.referenceSpacing);
result.diagnostics.initial_path=p;cuts=p.cusp_indices;count=numel(cuts)-1;phases=cell(count,1);q=[];
references=cell(count,1);gears=zeros(count,1);
for k=1:count,rows=cuts(k):cuts(k+1);references{k}=[p.x(rows) p.y(rows) p.theta(rows)];gears(k)=p.gear(rows(1));end
% Small parking phases avoid process-pool startup in the serial adapter.
for k=1:count,phases{k}=ritp.Phase(references{k},gears(k),c,o);end
result.diagnostics.phases=phases;
for k=1:count
 phase=phases{k};
 if ~phase.success,result.solver=phase.solver;result.solver.success=false;result.status.code=phase.code;result.status.message='A phase failed its QP, collision or physical steering check.';return;end
 q=ritp.Join(q,phase.trajectory,c.vehicle.wmax);
end
q=ritp.Join(q,[],c.vehicle.wmax);
result.trajectory=q;result.solver=struct('success',true,'phase_count',numel(phases),'path_iterations',cellfun(@(a)a.iterations,phases));
result.status=struct('success',true,'code','solved','message','Polynomial QPs and common physical checks passed; sampled full-body collision checks completed.');
end
