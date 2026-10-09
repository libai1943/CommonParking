function o=Config(v)
% TCST 2024 Section III, with numerical defaults from its ICRA 2019 reference.
o.length=.175;o.normalizedSteering=[1 .5 0 -.5 -1];o.delta=.04;o.mu=5;o.gamma=5;o.rho=1.25;
o.epsilon=2;o.metric=[1 1 v.turning_radius_min];o.seconds=180;o.iterations=100000;o.maxNodes=100000;
o.padding=3;o.sampleStep=.1;o.minimumStep=1e-5;o.margin=1e-8;o.outputSpacing=.05;
end
