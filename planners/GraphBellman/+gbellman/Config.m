function o=Config()
% Laurini, Consolini & Locatelli, TAC 2021, Model 1.
o.padding=5;o.spacing=.4;o.headingCount=48;o.yawCount=5;
o.stepDuration=.4;o.discount=.98;o.switchPenalty=10;
o.residualTolerance=1e-7;o.maximumSeconds=180;o.maximumPolicySteps=1000;
o.outputSpacing=.05;
end
