function o = Config()
% Printed TDR-OBCA formulation; unpublished choices fixed across cases.
o.nodes = 200;
o.weights = [1e-3,1,1,1,1e5,1]; % x, difference(x), u, prior-cycle u, goal, d
o.dualBeta = 1;
o.strictDistance = .01;
o.horizonFactor = 1.5;
o.speedStep = .1;
o.speedWeights = [1,1,10]; % acceleration, jerk, cruise-speed error
o.maxJerk = 2; % only the temporal initializer
o.maxIterations = 2000;
o.maxCpuSeconds = 30;
o.tolerance = 1e-8;
o.feasibilityTolerance = 1e-6;
o.search = struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
    'collisionStep',.04,'clearance',.01,'heuristicWeight',1,'switchPenalty',1.5, ...
    'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
    'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.qp = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-10,'OptimalityTolerance',1e-10,'MaxIterations',500);
end
