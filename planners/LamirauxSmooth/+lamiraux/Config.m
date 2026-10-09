function o=Config()
% Finite numerical choices for Lamiraux & Laumond (2001), Sections III-IV.
o.holonomicSeconds=40;o.holonomicIterations=20000;o.holonomicStep=1;o.goalBias=.1;o.padding=3;
o.geometricClearance=.12;o.certificateDepth=18;
o.subdivisionDepth=20;o.subdivisionCalls=5000;o.subdivisionSeconds=180;
o.curveDepth=18;o.curveClearance=.002;o.minimumParameterSpeed=1e-8;
o.cuspOffsetMax=12;o.cuspScales=[.125 .25 .5 1 2 4 8];
o.shortcutSeconds=60;o.shortcutIterations=10000;o.shortcutFailures=1000;o.improvementTolerance=1e-5;
o.outputSpacing=.02;o.seedBase=101000;
end
