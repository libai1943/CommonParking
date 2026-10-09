function options=Config()
% Sun et al., TITS 2022, Algorithm 1 and Table I.
options.nodes=200;options.circles=5;options.clearance=.1;
options.penalty=1e5;options.epsilon1=1e-3;options.epsilon2=1e-2;options.epsilon3=1;
options.initialRadius=4;options.minimumRadius=1e-3;
options.rho1=.2;options.rho2=.9;options.alpha=2.5;options.beta=2.5;
options.maxIterations=1500;options.maxSeconds=200;options.maxInnerIterations=100;
options.initialTime=20;options.maxTime=100;options.smallRadius=.1;
options.qpMaxIterations=300;options.qpTolerance=1e-9;
base=hacg.Config();options.search=base.search; % Paper Remark 3 permits Hybrid A*.
end
