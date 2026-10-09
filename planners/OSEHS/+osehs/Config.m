function o=Config()
% Unspecified numerical choices are centralized here and disclosed in README.
o.padding=6;o.circleAngles=16;o.minimumCircle=.02;o.maximumCircle=4;
o.circleSeconds=40;o.maximumCircles=120000;o.overlapFraction=.5;
o.stepFactors=[.5 .25 .125];o.minimumStep=.03;o.clusterFactor=1;
o.searchSeconds=180;o.maximumExpanded=100000;o.maximumNodes=600000;
o.wrongDirectionFactor=2;o.cuspPenalty=1.5;o.bidirectionalCuspPenalty=.15;
o.goalRange=6;o.outputSpacing=.05;
end
