function o=Config()
% Guo et al., TVT 2025, Table I. Vehicle limits come from the common case.
o.weights=[100 5 10];o.resamplingTime=.04;o.propagationTime=.5;o.trust=[1 1.5 1];
o.maximumOuterIterations=20;o.maximumTime=600;o.minimumTime=.1;o.maximumIntervals=2000;
o.dualDistance=1e-6;o.solverTolerance=1e-8;o.feasibilityTolerance=1e-6;o.maxIterations=2000;o.maxCpuSeconds=180;
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.12,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
end
