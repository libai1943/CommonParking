function options = Config()
% Shi et al. 2019: Algorithm 1, Tables 1 and 3.
options.nodes=15;
options.epsilon=10.^(-(1:8));options.safetyPseudodistance=.05;
options.objectiveTolerance=1e-5; % "small enough" is not quantified in the paper.
options.initialTime=20;options.maxTime=100;options.outputStep=.02;
options.maxIterations=3000;options.maxCpuSeconds=120;
end
