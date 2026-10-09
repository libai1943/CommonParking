function report = TestSetupCommonParking()
root=SetupCommonParking();before=path;restore=onCleanup(@()path(before));
SetupCommonParking();assert(strcmp(path,before),'Repeated setup changed an already complete path.');
folder=fullfile(root,'planners','LIOM');rmpath(folder);SetupCommonParking();
normalise=@(p)lower(strrep(string(p),'\','/'));
assert(ismember(normalise(folder),normalise(strsplit(path,pathsep))));
after=path;SetupCommonParking();assert(strcmp(path,after));
report=struct('passed',true,'idempotent_path',true,'missing_planner_restored',true);disp(report);
end
