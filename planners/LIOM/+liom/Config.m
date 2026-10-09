function options = Config()
% Li et al., TITS 2022, Table I and Eq. (10), (17)-(22), Algorithm 3.
options.nodes=201; % Benchmark refinement: 200 intervals; paper Table I: 50.
options.discPartitions=[3 1]; % Fixed three full-body covering discs; no refinement sequence.
options.corridorStep=.1;options.corridorExtent=10;options.clearance=.01;
options.nudgeStep=.16;options.nudgeLimit=8;
options.energyWeight=.01;options.penaltyWeight=1e9;options.feasibilityTolerance=1e-6;
options.outerIterations=10;options.maxIterations=200;options.maxCpuSeconds=2;
options.maxWallSeconds=10;options.maxTime=100;
options.search=struct('xyResolution',.32,'thetaResolution',2*pi/30,'step',1.5, ...
    'steeringSamples',5,'heuristicWeight',1.5,'reversePenalty',.5, ...
    'switchPenalty',.5,'steeringChangePenalty',.1,'maxExpanded',500,'maxSeconds',30, ...
    'analyticEvery',5,'variableStep',false,'returnPartial',true);
end
