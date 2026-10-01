function [initial,info] = Initialize(c,options)
raw=parking.SearchHybridAStar(c,options.search);info=struct('search',raw,'fallback_used',~raw.success);
start=[c.task.x0 c.task.y0 c.task.theta0];
if raw.success
    p=cp.PathFromArcs(start,raw.primitives,c.vehicle,.05);
else
    [tail,info.astar]=liom.GridAStar(c,raw.partial_endpoint(1:2),[c.task.xf c.task.yf],options.search.xyResolution);
    if isempty(tail),initial=[];info.success=false;return;end
    [q,gear,curvature,step]=parking.SamplePrimitives(start,raw.primitives,.05);
    for j=1:size(tail,1)-1
        delta=tail(j+1,:)-tail(j,:);length=norm(delta);count=max(1,ceil(length/.05));
        xy=tail(j,:)+(1:count)'/count.*delta;angle=atan2(delta(2),delta(1));
        q=[q;xy repmat(angle,count,1)];gear=[gear;ones(count,1)]; %#ok<AGROW>
        curvature=[curvature;zeros(count,1)];step=[step;repmat(length/count,count,1)]; %#ok<AGROW>
    end
    p=struct('s',[0;cumsum(abs(step))],'x',q(:,1),'y',q(:,2),'theta',unwrap(q(:,3)), ...
        'phi',atan(c.vehicle.lw*[curvature;curvature(end)]),'gear',gear, ...
        'cusp_indices',[1;find(diff(gear)~=0)+1;size(q,1)]);
end
seed=cp.EmptyResult('LIOM_initial_path',c.id,'path');seed.path=p;
reference=cpe.MakeReference(seed,c);initial=cpe.ReferenceAt(reference,linspace(0,reference.tf,options.nodes)',c.vehicle);
initial.omega=max(-c.vehicle.wmax,min(c.vehicle.wmax,gradient(initial.phi,initial.t)));
initial.a=max(-c.vehicle.amax,min(c.vehicle.amax,initial.a));
initial.v([1 end])=0;initial.phi([1 end])=0;initial.a([1 end])=0;initial.omega([1 end])=0;
info.success=true;
end
