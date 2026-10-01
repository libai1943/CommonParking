function result = Plan(c)
options=stc.Config();result=cp.EmptyResult('STC',c.id,'trajectory');
raw=parking.SearchHybridAStar(c,options.search);
result.solver.search_success=raw.success;
if ~raw.success,result.status.code='search_failed';result.status.message='Hybrid A* failed.';return;end
% The paper's initialization is Hybrid A* plus minimum-time longitudinal
% timing. It does not use the HA+CG smoother or insert stops at every arc.
seed=cp.EmptyResult('STC_initial_path',c.id,'path');
seed.path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],raw.primitives,c.vehicle,.05);
reference=cpe.MakeReference(seed,c);
initial=cpe.ReferenceAt(reference,linspace(0,reference.tf,options.nodes)',c.vehicle);
initial.omega=max(-c.vehicle.wmax,min(c.vehicle.wmax,gradient(initial.phi,initial.t)));
initial.a=max(-c.vehicle.amax,min(c.vehicle.amax,initial.a));
initial.v([1 end])=0;initial.phi([1 end])=0;initial.a([1 end])=0;initial.omega([1 end])=0;
history=cell(0,1);
for k=1:size(options.discPartitions,1)
    partition=options.discPartitions(k,:);discs=cp.CoveringDiscs(c.vehicle,partition(1),partition(2));
    [boxes,corridors]=cp.DiscCorridors(c,initial,discs,options);
    entry=struct('corridors',corridors,'solver',struct());
    if corridors.success
        [candidate,solver]=stc.Solve(c,initial,boxes,discs,options);
        entry.solver=solver;result.solver=solver;result.solver.disc_count=discs.count;
        result.solver.search_success=raw.success;
        if ~isempty(candidate),result.trajectory=candidate;end
        if solver.success
            result.status=struct('success',true,'code','solved','message',solver.message);
            history{end+1}=entry;break; %#ok<AGROW>
        end
    end
    history{end+1}=entry; %#ok<AGROW>
end
if ~result.status.success
    result.status.code='optimization_failed';
    result.status.message='No successful STC solve within the published covering-disc sequence.';
end
result.diagnostics=struct('search',raw,'initial',initial,'attempts',{history},'options',options);
end
