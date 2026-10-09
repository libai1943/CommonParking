function options=Config(vehicle)
% Banzhaf et al., IV 2018, Sections III-V; unspecified choices documented.
options.firstSolutionSeconds=5;options.refinementSeconds=3;options.goalBias=.05;options.gamma=6;
options.collisionStep=.1;options.hardClearance=.1;options.mapResolution=.075;options.inflationRadius=.25;
options.kappa=vehicle.kappa_max;
% |phi_dot| = L*|sigma|*|v|/(1+(L*kappa)^2).
% This bound respects the common steering-rate limit at ALL curvatures.
options.sigma=vehicle.wmax/(vehicle.lw*vehicle.vmax);
options.rho=.3905; % Paper Table III; common protocol does not bound this derivative.
options.weights=[1 2 1 1];options.seed=67000;
options.samplePadding=3;options.maxIterations=100000;
end
