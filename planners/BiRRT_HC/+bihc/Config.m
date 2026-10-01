function options=Config(vehicle)
% Banzhaf et al., ITSC 2017, Sections V-VI; unspecified choices documented.
options.seconds=6;options.goalBias=.05;options.gamma=6;
options.collisionStep=.1;options.hardClearance=.1;options.softClearance=.2;
options.kappa=vehicle.kappa_max;
% |phi_dot| = L*|sigma|*|v|/(1+(L*kappa)^2).
% This bound respects the common steering-rate limit at ALL curvatures.
options.sigma=vehicle.wmax/(vehicle.lw*vehicle.vmax);
options.weights=[1 2 1 1];options.seed=66000;
options.samplePadding=3;options.maxIterations=100000;
end
