function options = Config()
% ECC 2020 method; 200 nodes follow the supplied implementation (paper: 100).
options.nodes=200;
options.discPartitions=[2 1;4 2;6 3;8 4;10 5;12 6;16 8;20 10];
options.corridorStep=.1;options.corridorExtent=8;options.clearance=.01;
options.maxIterations=1500;options.maxCpuSeconds=90;options.maxTime=50;
options.search=struct('xyResolution',.2,'thetaResolution',.2,'step',.7, ...
    'steeringSamples',3,'heuristicWeight',3,'reversePenalty',1, ...
    'switchPenalty',5,'maxExpanded',100000,'maxSeconds',180,'variableStep',false);
end
