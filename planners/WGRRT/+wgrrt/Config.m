function o=Config()
o.holonomicSeconds=40;o.holonomicIterations=20000;o.holonomicStep=2;o.goalBias=.1;o.padding=3;
o.geometricClearance=.02;o.certificateDepth=18;o.waypointShortcutTrials=150;
o.neighborRadius=6;o.maximumRSPaths=3;o.addedNodesPerStage=100;o.stageSampleLimit=30000;
o.kinematicSeconds=180;o.gaussianWeights=[1 1 .25];o.meanFraction=.5;o.headingMetricScale=1;
o.shortcutSeconds=30;o.shortcutIterations=1500;o.shortcutFailures=150;o.improvementTolerance=1e-5;
o.seedBase=2017133;o.outputSpacing=.05;
end
