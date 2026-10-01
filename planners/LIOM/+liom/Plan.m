function result = Plan(c)
options=liom.Config();result=cp.EmptyResult('LIOM',c.id,'trajectory');
[seed,search]=liom.Initialize(c,options);result.solver.search_success=search.success;
if ~search.success,result.status.code='initialization_failed';result.status.message='Fault-tolerant search has no 2-D connection.';return;end
history=cell(0,1);polygons=parking.PolygonData(c);
for k=1:size(options.discPartitions,1)
    partition=options.discPartitions(k,:);discs=cp.CoveringDiscs(c.vehicle,partition(1),partition(2));
    t=c.task;q=[t.x0 t.y0 t.theta0;t.xf t.yf t.thetaf];o=discs.offsets;
    x=q(:,1)+cos(q(:,3))*o(:,1)'-sin(q(:,3))*o(:,2)';
    y=q(:,2)+sin(q(:,3))*o(:,1)'+cos(q(:,3))*o(:,2)';
    gap=cp.BoxObstacleDistance([x(:) x(:) y(:) y(:)],polygons)-discs.radius-options.clearance;
    if any(gap<0)
        history{end+1}=struct('disc_count',discs.count,'code','endpoint_cover_blocked');continue; %#ok<AGROW>
    end
    current=seed;
    for iteration=1:options.outerIterations
        [boxes,corridors]=liom.Corridors(c,current,discs,options);
        if ~corridors.success
            history{end+1}=struct('disc_count',discs.count,'code','corridor_failed','iteration',iteration);break; %#ok<AGROW>
        end
        [candidate,solver]=liom.Solve(c,current,boxes,discs,options);
        history{end+1}=struct('disc_count',discs.count,'iteration',iteration,'corridors',corridors,'solver',solver); %#ok<AGROW>
        result.solver=solver;result.solver.disc_count=discs.count;result.solver.outer_iterations=iteration;
        result.solver.search_success=true;result.solver.fault_tolerant_fallback=search.fallback_used;
        if ~isempty(candidate),result.trajectory=rmfield(candidate,{'cx','cy'});end
        if ~solver.success,break;end
        if solver.infeasibility<options.feasibilityTolerance
            result.status=struct('success',true,'code','solved','message','Native NLP solved and paper infeasibility tolerance satisfied.');break;
        end
        current=candidate;
    end
    if result.status.success,break;end
end
if ~result.status.success
    result.status.code='optimization_failed';result.status.message='LIOM did not meet native solve and paper feasibility criteria within the configured attempts.';
end
result.diagnostics=struct('initialization',search,'initial',seed,'attempts',{history},'options',options);
end
