function o=Config()
% Jhang, Lian & Hao, CASE 2020. Unspecified numerical choices are documented.
o.seconds=60;o.maximumIterations=20000;o.rewireRadius=10;o.padding=5;o.goalBias=.05;
o.weights=[1 1 5 1];o.sideMirrorMargin=.19;o.buffer=0;
o.collisionSpacing=.05;o.interpolationSpacing=.5;o.roiPadding=2;o.outputSpacing=.05;
o.seedBase=2020105;
end
