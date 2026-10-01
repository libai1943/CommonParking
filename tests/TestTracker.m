function report = TestTracker()
% End-to-end exact straight maneuver and obstacle-blind tracking checks.
SetupCommonParking();cfg=BenchmarkConfig();c=LoadCase(1);
c.task.x0=0;c.task.y0=0;c.task.theta0=0;c.task.xf=4;c.task.yf=0;c.task.thetaf=4*pi;
c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[100 101 101 100 100],'y',[100 100 101 101 100])};
r=cp.EmptyResult('test',1,'path');r.status.success=true;r.computation_time_s=0;
r.path=cp.PathFromArcs([0 0 0],[4 0],c.vehicle,.05);
e=EvaluateResult(r,c);
assert(e.success,e.message);assert(e.metrics.terminal_reached);
assert(e.metrics.collision_percent==0);
assert(e.debug.integration.max_pose_defect<cfg.evaluation.integration_pose_tolerance);
assert(abs(e.metrics.execution_time_s-4)<.03);
assert(abs(e.debug.execution.v(end))<1e-6);
% Obstacles are checked only after tracking, and never enter its equations.
blocked=c;blocked.obstacle.obs={struct('x',[1 3 3 1 1],'y',[-1 -1 1 1 -1])};
[m,~]=cpe.Metrics(e.debug.execution,blocked,cfg);
assert(m.collision_percent>0);
tr=r;tr.kind='trajectory';tr.path=[];tr.trajectory=e.debug.execution;
assert(cpe.ValidateResult(tr,c,cfg).valid);
tr.trajectory.t(2)=tr.trajectory.t(1);assert(~cpe.ValidateResult(tr,c,cfg).valid);
report=struct('passed',true,'duration_s',e.metrics.execution_time_s, ...
    'pose_defect',e.debug.integration.max_pose_defect,'terminal_error',e.debug.terminal_error, ...
    'blocked_collision_percent',m.collision_percent);
disp(report);
end
