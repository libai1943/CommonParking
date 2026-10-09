function o=Config()
% Zhang et al., IET ITS 2021: RK4 multiple shooting and protection frames.
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.12,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
o.intervals=40;o.maximumTime=80;o.minimumTime=.1;o.jerkLimit=.6;
o.alphaTolerance=1e-3;o.maximumOuterIterations=8;o.maximumAlpha=4;
o.segmentThreshold=1e-8;o.feasibilityTolerance=1e-6;o.solverTolerance=1e-8;
o.maxIterations=2000;o.maxCpuSeconds=180;
end
