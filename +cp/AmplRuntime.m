function runtime = AmplRuntime()
% Configure a legally obtained local AMPL/Ipopt installation once, outside git:
% setpref('CommonParking','AmplDirectory','C:\path\to\runtime')
folder=getenv('COMMONPARKING_AMPL_DIR');
if isempty(folder),folder=getpref('CommonParking','AmplDirectory','');end
if ispc,exe='.exe';else,exe='';end
runtime=struct('ampl',fullfile(folder,['ampl',exe]),'ipopt',fullfile(folder,['ipopt',exe]));
if isempty(folder)||~isfile(runtime.ampl)||~isfile(runtime.ipopt)
    error('CommonParking:MissingRuntime','Set COMMONPARKING_AMPL_DIR or the CommonParking/AmplDirectory MATLAB preference to your AMPL/Ipopt installation. Runtime binaries are not distributed.');
end
end
