function options = Config()
% Li & Shao, KBS 2015, three collocation stages per finite element.
options.elements=66; % 199 unique states; paper examples use 20 elements.
options.areaMargin=.01; % m^2, numerical replacement for strict outside test.
options.maxTime=100;options.maxIterations=3000;options.maxCpuSeconds=120;
options.referenceTimeMultiplier=1.5;
options.search=struct('xyResolution',.2,'thetaResolution',.2,'step',.7, ...
    'steeringSamples',3,'heuristicWeight',3,'reversePenalty',1, ...
    'switchPenalty',5,'maxExpanded',100000,'maxSeconds',180,'variableStep',false);
end
