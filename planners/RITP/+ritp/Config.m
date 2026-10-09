function o=Config()
% Paper equations take precedence over the later author's Python variant.
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.12,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.degree=5;o.samplesPerMetre=20;o.minimumSamples=15;o.smoothingOrder=2;
o.beta=1.2;o.maximumIterations=10;o.pathWeights=[50 .1 .5];
o.velocityWeights=[1 .5 .3];o.velocitySampleRatio=.7;o.timeRatio=2;
o.referenceSpacing=.1;o.derivativeTolerance=1e-7;o.feasibilityTolerance=1e-7;
o.maximumWorkers=4;
end
