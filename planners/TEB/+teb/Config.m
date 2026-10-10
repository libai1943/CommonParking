function o=Config()
% Roesmann et al., RAS 2017 backbone; common five-state static adapter.
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.01,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.spacing=.3;o.dt=.3;o.hysteresis=.03;o.minimumPoses=3;o.maximumPoses=500;
o.planningCycles=10;o.resizeCalls=4;o.lmIterations=5;o.lmTrials=12;o.maxSeconds=180;
o.samples=15;o.sampleTrials=1500;o.graphWidth=6;o.graphLengthScale=1.1;
o.forwardCosine=cos(pi/3);o.maximumClasses=3;o.maximumDfsVisits=20000;
o.signatureTolerance=1e-7;o.seedBase=201701;o.minimumDt=1e-5;
o.clearance=.01;
o.dynamicsWeight=1e5;o.limitsWeight=1e3;o.obstacleWeight=1e4;
o.gradientTolerance=1e-7;o.stepTolerance=1e-9;o.finiteDifferenceStep=1e-6;
end
