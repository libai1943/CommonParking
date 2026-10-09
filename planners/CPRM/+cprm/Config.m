function o=Config()
% Finite numerical choices for Song & Amato, IROS 2001.
o.controlNodes=400;o.controlTrials=20000;o.neighbors=16;o.padding=3;
o.coarseCurvature=2;o.queryNeighbors=40;o.queryIterations=3000;o.seconds=120;
o.splineIterations=20;o.splineImprovement=1e-5;o.minimumSpeed=1e-8;
o.collisionClearance=.002;o.collisionDepth=18;o.outputSpacing=.02;o.seedBase=112000;
end
