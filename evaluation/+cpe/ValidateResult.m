function check = ValidateResult(r,c,cfg)
% Structural screening only. Signed coordinates, heading and reverse speed
% are legitimate; complex values are rejected rather than silently truncated.
check = struct('valid',false,'code','invalid_result','message','');
try
    require(isstruct(r)&&isscalar(r),'Result must be a scalar structure.');
    require(isfield(r,'schema_version')&&strcmp(r.schema_version,'CommonParking-result-1'),'Unsupported result schema.');
    require(isfield(r,'kind')&&(ischar(r.kind)||isstring(r.kind))&&any(strcmp(r.kind,{'path','trajectory'})),'kind must be path or trajectory.');
    require(isfield(r,'case_id')&&isequal(r.case_id,c.id),'Result belongs to another case.');
    require(isfield(r,'status')&&isstruct(r.status)&&isfield(r.status,'success'),'Missing planner success flag.');
    flag = r.status.success;
    require((islogical(flag)||isnumeric(flag))&&isreal(flag)&&isscalar(flag)&&ismember(flag,[0 1]),'Invalid success flag.');
    if ~flag
        check.code='planner_failed';check.message='Planner reported failure.';return;
    end
    require(isfield(r,'computation_time_s')&&realScalar(r.computation_time_s)&&r.computation_time_s>=0,'Invalid computation time.');
    z = r.(r.kind);
    require(isstruct(z)&&isscalar(z),'Missing solution structure.');
    require(isfield(z,'x'),'Missing x.');
    n = numel(z.x);
    require(n>=2&&n<=cfg.evaluation.max_samples,'Invalid number of samples.');
    for name={'x','y','theta','phi'}
        field=name{1}; require(isfield(z,field)&&realVector(z.(field),n),['Invalid ',field,'.']);
    end
    require(all(hypot(z.x-c.task.x0,z.y-c.task.y0)<1000),'Reference leaves the local domain.');
    require(all(abs(z.phi)<pi/2),'Steering must lie strictly inside (-pi/2,pi/2).');
    if strcmp(r.kind,'path')
        require(isfield(z,'s')&&realVector(z.s,n)&&abs(z.s(1))<1e-9&&all(diff(z.s)>0),'Invalid path mileage.');
        require(z.s(end)<=cfg.evaluation.max_reference_length_m,'Path exceeds resource limit.');
        require(isfield(z,'gear')&&realVector(z.gear,n-1)&&all(ismember(z.gear,[-1 1])),'Invalid path gear intervals.');
        require(all(hypot(diff(z.x),diff(z.y))<=diff(z.s)+1e-6),'Chord exceeds stated arc length.');
        cuts=[1;find(diff(z.gear)~=0)+1;n];
        require(isfield(z,'cusp_indices')&&isequal(z.cusp_indices(:),cuts),'Cusp indices must include both endpoints and every gear change.');
        for j=1:numel(cuts)-1
            steps=diff(z.s(cuts(j):cuts(j+1)));
            require(max(steps)<=cfg.output.path_spacing_max_m+1e-8&&max(steps)-min(steps)<1e-8,'Path spacing is not uniform within a gear run or exceeds 0.05 m.');
        end
        if isfield(z,'geometry')
            g=z.geometry;
            require(isstruct(g)&&isfield(g,'type')&&strcmp(g.type,'piecewise_circular'),'Unknown optional geometry.');
            require(isfield(g,'primitives')&&isnumeric(g.primitives)&&isreal(g.primitives)&&size(g.primitives,2)==2&&all(isfinite(g.primitives),'all')&&all(abs(g.primitives(:,1))>0),'Invalid exact arc geometry.');
            require(abs(sum(abs(g.primitives(:,1)))-z.s(end))<1e-7,'Exact arc length disagrees with samples.');
            require(isfield(g,'start')&&isnumeric(g.start)&&isreal(g.start)&&numel(g.start)==3&&all(isfinite(g.start)),'Invalid exact arc start.');
            [q,kap,d]=cp.ArcPose(g.start,g.primitives,z.s);
            angular=atan2(sin(q(:,3)-z.theta),cos(q(:,3)-z.theta));
            require(max(abs([q(:,1)-z.x;q(:,2)-z.y;angular]))<1e-6,'Exact geometry disagrees with exported poses.');
            require(max(abs(atan(c.vehicle.lw*kap)-z.phi))<1e-6,'Exact geometry disagrees with steering.');
            [~,~,edgeGear]=cp.ArcPose(g.start,g.primitives,(z.s(1:end-1)+z.s(2:end))/2);
            require(isequal(edgeGear,z.gear),'Exact geometry disagrees with gear.');
        end
    else
        require(isfield(z,'t')&&realVector(z.t,n)&&abs(z.t(1))<1e-9&&all(diff(z.t)>0),'Time must start at zero and strictly increase.');
        require(z.t(end)<=cfg.evaluation.max_reference_time_s,'Trajectory exceeds resource limit.');
        for name={'v','a','omega'}
            field=name{1};require(isfield(z,field)&&realVector(z.(field),n),['Invalid ',field,'.']);
        end
        require(abs(z.v(1))<1e-6&&abs(z.v(end))<1e-6,'Trajectory must start and finish at rest.');
    end
    position=[z.x(1)-c.task.x0;z.y(1)-c.task.y0;z.x(end)-c.task.xf;z.y(end)-c.task.yf];
    angles=[z.theta(1)-c.task.theta0;z.theta(end)-c.task.thetaf];
    if max(abs(position))>cfg.evaluation.gross_position_error_m || max(abs(atan2(sin(angles),cos(angles))))>cfg.evaluation.gross_heading_error_rad
        check.code='gross_task_mismatch';check.message='Reference misses the task by more than 1 m or 30 degrees.';return;
    end
    check.valid=true;check.code='valid';check.message='Passed structural and gross-task screening.';
catch problem
    check.message=problem.message;
end
end
function ok=realScalar(x)
ok=isnumeric(x)&&isreal(x)&&isscalar(x)&&isfinite(x);
end
function ok=realVector(x,n)
ok=isnumeric(x)&&isreal(x)&&iscolumn(x)&&numel(x)==n&&all(isfinite(x));
end
function require(condition,message)
if ~condition,error('CommonParking:InvalidResult','%s',message);end
end
