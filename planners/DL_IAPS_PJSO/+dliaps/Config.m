function o=Config()
% Paper equations/Algorithm 1; unspecified numerical choices are explicit.
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.12,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.searchCurvatureFraction=.8;o.minimumPoints=12;
o.spacing=.1;o.initialBubble=2;o.collisionShrink=.9;o.maxCollisionIterations=100;
o.initialPenalty=10;o.penaltyFactor=10;o.maxPenaltyIterations=7;o.maxSubIterations=1000;
o.initialTrust=1;o.maximumTrust=2;o.trustGrow=2;o.trustShrink=.5;o.rho=.1;
o.xTolerance=1e-6;o.fTolerance=1e-6;o.constraintTolerance=1e-5;
o.maxOptimizationSeconds=180;o.maxPoints=1500;
o.timeStep=.2;o.horizonFactor=1.5;o.maxJerk=1;o.maxLateralAcceleration=1;
o.distanceWeight=10;o.accelerationWeight=1;o.jerkWeight=1;o.outputTimeStep=.02;
o.qp=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
 'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'StepTolerance',1e-12,'MaxIterations',300);
end
