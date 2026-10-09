function options=Config(vehicle)
% Bergman, Ljungqvist & Axehill, TIV 2021: complete car model, Eq. (14).
options.grid=1;options.headings=atan2([0 1 1 2 1 2 1 1 0 -1 -1 -2 -1 -2 -1 -1],[1 2 1 1 0 -1 -1 -2 -1 -2 -1 -1 0 1 1 2]);
options.headingSteps=4;options.parallelOffsets=1:3;
options.gamma=1;options.steeringRatePerMetre=vehicle.wmax/vehicle.vmax;
options.steeringAccelerationPerMetre2=40;
options.primitiveIntervals=60; % Improvement retains these intervals per phase.
options.nearestEndpoints=16;options.heuristicSideLength=40;
options.maxLength=200;options.maxIterations=2000;options.maxCpuSeconds=240;
options.barrierStrategy='monotone';
options.tolerance=1e-9;options.searchSeconds=180;options.searchExpanded=500000;
options.clearance=.01;options.collisionSpacing=.05;options.searchPadding=10;
end
