function EnsureNative()
% Pre-built cache or explicit one-time BuildHCRSteer compilation.
if exist('hcr_steer_mex','file')==3,return;end
folder=getenv('COMMONPARKING_HCR_DIR');
if isempty(folder),folder=fullfile(tempdir,'CommonParking','hcr-steer',computer('arch'));end
if isfile(fullfile(folder,['hcr_steer_mex.',mexext])),addpath(folder);end
if exist('hcr_steer_mex','file')~=3
    error('CommonParking:HCBuildRequired','Run BuildHCRSteer once with a configured MATLAB C++ compiler, or point COMMONPARKING_HCR_DIR to a compiled cache.');
end
end
