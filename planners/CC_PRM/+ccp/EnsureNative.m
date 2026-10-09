function EnsureNative()
if exist('cc_steer_mex','file')==3,return;end
folder=getenv('COMMONPARKING_CC_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','cc-steer',computer('arch'));end
if isfolder(folder),addpath(folder);end
assert(exist('cc_steer_mex','file')==3,'Run BuildCCSteer once with a configured C++ compiler, or set COMMONPARKING_CC_DIR.');
end
