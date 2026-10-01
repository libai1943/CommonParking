function caseData = LoadCase(caseId)
%LOADCASE Load one immutable scene. Angles are radians, rear axle is the origin.
arguments
    caseId (1,1) double {mustBeInteger,mustBeInRange(caseId,1,12)}
end
root = SetupCommonParking();
stored = load(fullfile(root,'cases',sprintf('Case%02d.mat',caseId)),'caseData');
caseData = stored.caseData;
cfg = BenchmarkConfig();
caseData.vehicle = cfg.vehicle;
end
