function o=Config()
% Static paper formulation; path export explicitly discards the native time law.
o.gearSpeed=.05;o.pieceDuration=1;o.samplesPerPiece=16;o.maximumPieces=80;
o.timeWeight=500;o.feasibilityWeight=2500;o.obstacleWeight=1000;o.lateralAcceleration=5;
o.smoothing=1e-4;o.corridorStep=.1;o.corridorExtent=2;
o.maximumIterations=4000;o.maximumSeconds=180;o.maximumObjective=50000;
o.search=struct('xyResolution',.2,'thetaResolution',pi/36,'step',.5, ...
 'collisionStep',.04,'clearance',.12,'heuristicWeight',1,'switchPenalty',1.5, ...
 'reversePenalty',.05,'maxExpanded',100000,'maxSeconds',180,'analyticEvery',5, ...
 'steeringSamples',3,'variableStep',true,'stepScale',.1,'maximumStep',1);
end
