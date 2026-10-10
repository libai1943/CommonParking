function o=Config()
% Fan, Murgovski and Liang, TITS 2024, equations (12), (26), (27).
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.01,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.intervals=200;o.minimumTime=.1;o.maximumTime=600;
o.timeWeight=1;o.accelerationWeight=100;o.steeringRateWeight=200;
o.supportTolerance=1e-5;
o.feasibilityTolerance=1e-6;o.solverTolerance=1e-8;o.maxIterations=2000;o.maxCpuSeconds=60;
end
