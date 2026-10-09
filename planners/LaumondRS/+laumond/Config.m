function options=Config()
% Laumond et al. 1994, Section III. Finite implementation resource choices.
options.holonomicSeconds=40;options.holonomicIterations=20000;
options.holonomicStep=1;options.goalBias=.1;options.padding=3;
options.geometricClearance=.12;options.certificateDepth=18;
options.subdivisionDepth=22;options.subdivisionCalls=12000;options.subdivisionSeconds=90;
options.shortcutSeconds=30;options.shortcutIterations=1500;options.shortcutFailures=150;
options.improvementTolerance=1e-5;options.seedBase=94000;
end
