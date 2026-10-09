function o=Config()
% Triangle21 settings; vehicle and task data come from CommonParking.
o.nodes=200;o.areaMargin=.01; % square metres, not distance inflation
o.maxTime=100;o.maxIterations=3000;o.maxCpuSeconds=60;
o.search=struct('xyResolution',.2,'thetaResolution',.2,'step',.7, ...
 'steeringSamples',3,'heuristicWeight',3,'reversePenalty',1,'clearance',0, ...
 'switchPenalty',5,'maxExpanded',100000,'maxSeconds',180,'variableStep',false);
end
