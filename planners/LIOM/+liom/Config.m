function options = Config()
% Li et al., TITS 2022, Table I and Eq. (10), (17)-(22), Algorithm 3.
options.nodes=51; % N_FE=50; Section III explicitly uses indices 0..N_FE.
options.discPartitions=[2 1;4 2;6 3;8 4;10 5;12 6;16 8;20 10];
options.corridorStep=.1;options.corridorExtent=10;options.clearance=.01;
options.nudgeStep=.16;options.nudgeLimit=8;
options.energyWeight=.01;options.penaltyWeight=1e9;options.feasibilityTolerance=1e-6;
options.outerIterations=10;options.maxIterations=3000;options.maxCpuSeconds=90;options.maxTime=100;
options.search=struct('xyResolution',.32,'thetaResolution',2*pi/30,'step',1.5, ...
    'steeringSamples',5,'heuristicWeight',1.5,'reversePenalty',.5, ...
    'switchPenalty',.5,'steeringChangePenalty',.1,'maxExpanded',500,'maxSeconds',30, ...
    'analyticEvery',5,'variableStep',false,'returnPartial',true);
end
