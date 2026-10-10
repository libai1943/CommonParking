function result=Plan(c)
o=dliaps.Config();result=cp.EmptyResult('DL_IAPS_PJSO',c.id,'trajectory');
previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
seedCase=c;seedCase.vehicle.kappa_max=o.searchCurvatureFraction*c.vehicle.kappa_max;
seedCase.vehicle.turning_radius_min=1/seedCase.vehicle.kappa_max;
seedCase.vehicle.phimax=atan(seedCase.vehicle.kappa_max*c.vehicle.lw);
raw=parking.SearchHybridAStar(seedCase,o.search);result.solver.search_success=raw.success;
result.diagnostics=struct('options',o,'search',raw,'maneuvers',{{}});
if ~raw.success,result.status.code='search_failed';result.status.message='Hybrid A* initialization failed.';return;end
t=c.task;path=cp.PathFromArcs([t.x0 t.y0 t.theta0],raw.primitives,c.vehicle,o.spacing);
timer=tic;trajectory=[];
for run=1:numel(path.cusp_indices)-1
 ids=path.cusp_indices(run):path.cusp_indices(run+1);s=path.s(ids);
 if numel(ids)<o.minimumPoints,s=linspace(s(1),s(end),o.minimumPoints)';end
 if numel(s)>o.maxPoints,result.status.code='path_resource_limit';return;end
 reference=cp.ArcPose(path.geometry.start,raw.primitives,s);gear=path.gear(ids(1));
 [p,details]=dliaps.Smooth(reference,gear,c,o,timer);details.clearance=o.clearance;
 if ~details.success&&o.clearance>0&&toc(timer)<o.maxOptimizationSeconds
  firstAttempt=struct('code',details.code,'clearance',o.clearance);
  local=o;local.clearance=0;[p,details]=dliaps.Smooth(reference,gear,c,local,timer);
  details.clearance=0;details.preferred_margin_attempt=firstAttempt;
 end
 entry=struct('path',p,'smoothing',details,'speed',[],'speed_solver',[]);
 result.diagnostics.maneuvers{run}=entry;
 if ~details.success,result.status.code=['path_' details.code];result.status.message=sprintf('DL-IAPS failed in maneuver %d.',run);return;end
 [speed,speedInfo]=dliaps.Speed(p,c.vehicle,o);entry.speed=speed;entry.speed_solver=speedInfo;result.diagnostics.maneuvers{run}=entry;
 if ~speedInfo.success,result.status.code='speed_qp_failed';result.status.message=sprintf('PJSO failed in maneuver %d.',run);return;end
 q=dliaps.Trajectory(p,speed,c.vehicle,o);
 trajectory=dliaps.Join(trajectory,q,c.vehicle.wmax);

end
trajectory=dliaps.Join(trajectory,[],c.vehicle.wmax);
result.trajectory=trajectory;result.solver.success=true;
result.status=struct('success',true,'code','solved','message','DL-IAPS and every piecewise-jerk QP passed native criteria; execution evaluation is separate.');
end
