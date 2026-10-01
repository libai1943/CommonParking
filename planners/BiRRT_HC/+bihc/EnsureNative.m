function EnsureNative()
% Pre-built cache or explicit one-time BuildHCSteer compilation.
if exist('hc_steer_mex','file')==3,return;end
folder=getenv('COMMONPARKING_HC_DIR');
if isempty(folder),folder=fullfile(tempdir,'CommonParking','hc-steer',computer('arch'));end
if isfile(fullfile(folder,['hc_steer_mex.',mexext])),addpath(folder);end
if exist('hc_steer_mex','file')~=3
    error('CommonParking:HCBuildRequired','Run BuildHCSteer once with a configured MATLAB C++ compiler, or point COMMONPARKING_HC_DIR to a compiled cache.');
end
end
