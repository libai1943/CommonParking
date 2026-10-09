function result=Plan(c)
o=eta3.Config();result=cp.EmptyResult('Eta3',c.id,'path');
previous=maxNumCompThreads(1);cleanup=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
raw=parking.SearchHybridAStar(c,o.search);result.diagnostics=struct('options',o,'search',raw);
if ~raw.success,result.status.code='initialization_failed';result.status.message='The disclosed Hybrid A* initializer failed.';return;end
path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],raw.primitives,c.vehicle,.05);
cusps=path.cusp_indices;h=numel(cusps)-1;gears=path.gear(cusps(1:end-1));
if h>o.maximumManeuvers,result.status.code='maneuver_resource_limit';return;end
lengths=diff(path.s(cusps));etas=reshape([lengths lengths]',[],1);
interior=[path.x(cusps(2:end-1)),path.y(cusps(2:end-1)),path.theta(cusps(2:end-1)),zeros(h-1,1)];
z0=[zeros(2*h,1);etas;reshape(interior',[],1)];
allxy=[c.task.x0 c.task.y0;c.task.xf c.task.yf];
for j=1:c.obstacle.num_obs,ob=c.obstacle.obs{j};allxy=[allxy;ob.x(:) ob.y(:)];end %#ok<AGROW>
lo=min(allxy)-6;hi=max(allxy)+6;
lower=[-o.curvatureDerivativeMax*ones(2*h,1);o.minimumEta*ones(2*h,1);reshape([repmat(lo,h-1,1),interior(:,3)-pi,-c.vehicle.kappa_max*ones(h-1,1)]',[],1)];
upper=[o.curvatureDerivativeMax*ones(2*h,1);max(2,4*etas+2);reshape([repmat(hi,h-1,1),interior(:,3)+pi,c.vehicle.kappa_max*ones(h-1,1)]',[],1)];
[z,solver]=eta3.Optimize(z0,lower,upper,c,gears,o);pieces=eta3.Decode(z,c.task,gears);
result.solver=solver;result.diagnostics.initial_variables=z0;result.diagnostics.variables=z;result.diagnostics.gears=gears;
result.diagnostics.lower=lower;result.diagnostics.upper=upper;
if solver.success
 for k=1:numel(pieces),pieces(k)=eta3.Prepare(pieces(k));end
 result.path=eta3.Export(pieces,c.vehicle,o.outputSpacing);result.diagnostics.pieces=pieces;
 result.status=struct('success',true,'code','solved','message','The eta3 spline NLP passed its native flag and 100-interval sampled constraints.');
else
 result.status.code='optimization_failed';result.status.message=solver.message;
end
end
