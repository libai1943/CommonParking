function o=Config()
% Weighted polynomial QPs with explicit common-vehicle adapters.
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.01,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.searchCurvatureFraction=.8;o.degree=9;o.maximumDegree=10;o.samplesPerMetre=20;o.minimumSamples=15;
o.beta=1.2;o.maximumIterations=30;o.pathWeights=[50 .1 .5];
o.velocityWeights=[1 .5 .3];o.velocitySampleRatio=.7;o.timeRatio=2;
o.referenceSpacing=.1;o.derivativeTolerance=1e-7;o.feasibilityTolerance=1e-7;
end
