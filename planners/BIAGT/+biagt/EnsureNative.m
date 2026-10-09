function EnsureNative()
if exist('biagt_rs_mex','file')~=3
 folder=getenv('COMMONPARKING_BIAGT_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','biagt',computer('arch'));end
 if isfolder(folder),addpath(folder);end
 assert(exist('biagt_rs_mex','file')==3,'Run BuildBIAGT once.');
end
ccp.EnsureNative();
end
