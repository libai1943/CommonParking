function cfg = BenchmarkConfig()
%BENCHMARKCONFIG Versioned, common physical and output conventions.
cfg.version = 'CommonParking-1.0';
cfg.vehicle = struct('lw',2.8,'lf',0.96,'lr',0.929,'lb',1.942, ...
    'v_forward',2.5,'v_reverse',2.5,'a_accel',1.0,'a_brake',1.0, ...
    'phimax',0.7,'wmax',0.5);
cfg.vehicle.length = cfg.vehicle.lw+cfg.vehicle.lf+cfg.vehicle.lr;
cfg.vehicle.vmax = 2.5;
cfg.vehicle.amax = 1.0;
cfg.vehicle.kappa_max = tan(cfg.vehicle.phimax)/cfg.vehicle.lw;
cfg.vehicle.turning_radius_min = 1/cfg.vehicle.kappa_max;
cfg.vehicle.reference_point = 'rear_axle_midpoint';
cfg.output.path_spacing_max_m = 0.05;
cfg.output.numeric_tolerance = 1e-9;
cfg.evaluation.limit_relaxation = 0.001;
cfg.evaluation.frame_dt_s = 0.001;
cfg.evaluation.terminal_xy_m = 0.01;
cfg.evaluation.terminal_heading_rad = pi/180;
cfg.evaluation.gross_position_error_m = 1.0;
cfg.evaluation.gross_heading_error_rad = pi/6;
cfg.evaluation.max_reference_time_s = 600;
cfg.evaluation.max_reference_length_m = 1000;
cfg.evaluation.max_samples = 2000000;
cfg.evaluation.tracker_nodes = 2001;
cfg.evaluation.tracker_max_nodes = 8001;
cfg.evaluation.tracker_tolerance = 1e-8;
cfg.evaluation.integration_pose_tolerance = 1e-4;
cfg.evaluation.max_iterations = 2000;
cfg.evaluation.max_cpu_seconds = 180;
% Free duration tracks reference phase. This small, published regularizer
% resolves time-scale ambiguity; it is NOT a benchmark score.
cfg.evaluation.time_regularization = 1e-4;
cfg.evaluation.time_scale_bounds = [0.5 3.0];
cfg.evaluation.control_smoothing = 1e-4;
cfg.evaluation.effort_weight = 10;
cfg.evaluation.steering_weight = 10;
cfg.evaluation.gear_change_weight = 5;
end
