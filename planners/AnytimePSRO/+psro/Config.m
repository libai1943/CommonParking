function o=Config()
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.12,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.nodes=200;o.minimumStep=.001;o.maximumStep=.3;o.gapTolerance=.01*o.nodes;
o.accelerationWeight=.1;o.steeringRateWeight=.1;o.areaExcess=1e-6;
o.maximumOuterIterations=10;o.solverTolerance=1e-8;o.feasibilityTolerance=1e-6;
o.maxIterations=2000;o.maxCpuSeconds=180;
end
