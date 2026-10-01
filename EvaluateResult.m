function evaluation = EvaluateResult(result,caseData)
%EVALUATERESULT Validate, time a path if needed, track, then measure independently.
% evaluation = EvaluateResult(result,LoadCase(result.case_id))
SetupCommonParking();cfg=BenchmarkConfig();
evaluation=struct('schema_version','CommonParking-evaluation-1','case_id',caseData.id, ...
    'success',false,'code','not_evaluated','message','', ...
    'metrics',struct('collision_percent',NaN,'terminal_reached',NaN, ...
        'execution_time_s',NaN,'control_effort_integral',NaN, ...
        'steering_integral',NaN,'gear_changes',NaN,'smoothness_cost',NaN), ...
    'screening',struct(),'solver',struct(),'debug',struct(),'protocol',cfg.evaluation);
evaluation.screening=cpe.ValidateResult(result,caseData,cfg);
if ~evaluation.screening.valid
    evaluation.code=evaluation.screening.code;evaluation.message=evaluation.screening.message;return;
end
try
    ref=cpe.MakeReference(result,caseData);evaluation.debug.reference=ref;
    if ref.tf>cfg.evaluation.max_reference_time_s
        evaluation.code='reference_resource_limit';evaluation.message='Timed path exceeds 600 s.';return;
    end
    nodes=cfg.evaluation.tracker_nodes;
    history=cell(0,1);
    while true
        [native,solver]=cpe.SolveTracker(ref,caseData,cfg,nodes);
        history{end+1}=solver; %#ok<AGROW>
        evaluation.solver=solver;evaluation.debug.tracker_history=history;
        if ~solver.success||isempty(native)
            evaluation.code='tracking_failed';evaluation.message=solver.message;return;
        end
        [execution,integration]=cpe.ExecuteControls(native,caseData,cfg);
        evaluation.debug.native=native;evaluation.debug.integration=integration;
        if integration.max_pose_defect<=cfg.evaluation.integration_pose_tolerance && integration.max_dynamic_violation<=5e-7,break;end
        if nodes>=cfg.evaluation.tracker_max_nodes
            evaluation.code='tracking_discretization_failed';
            evaluation.message='Independent integration does not reproduce the collocation solution within 0.1 mm/rad tolerance.';return;
        end
        nodes=min(2*(nodes-1)+1,cfg.evaluation.tracker_max_nodes);
    end
    [metrics,collision]=cpe.Metrics(execution,caseData,cfg);
    evaluation.metrics=metrics;evaluation.debug.execution=execution;
    evaluation.debug.collision_mask=collision;
    evaluation.debug.terminal_error=[execution.x(end)-caseData.task.xf,execution.y(end)-caseData.task.yf, ...
        atan2(sin(execution.theta(end)-caseData.task.thetaf),cos(execution.theta(end)-caseData.task.thetaf))];
    evaluation.success=true;evaluation.code='evaluated';
    evaluation.message='Tracking succeeded. Collision and terminal attainment are separate measured outcomes.';
catch problem
    evaluation.code='evaluation_exception';evaluation.message=problem.message;
    evaluation.debug.exception=getReport(problem,'extended','hyperlinks','off');
end
end
