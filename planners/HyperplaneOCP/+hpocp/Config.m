function o=Config()
% Fan, Murgovski and Liang, TITS 2024, equations (12), (26), (27).
o.intervals=200;o.initialTime=50;o.minimumTime=.1;o.maximumTime=600;
o.timeWeight=1;o.accelerationWeight=100;o.steeringRateWeight=200;
o.normalMinimum=.01;o.hyperplaneFraction=.75;
o.feasibilityTolerance=1e-6;o.solverTolerance=1e-8;o.maxIterations=2000;o.maxCpuSeconds=180;
end
