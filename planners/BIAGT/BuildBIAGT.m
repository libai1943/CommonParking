function folder = BuildBIAGT(zigExecutable)
%BUILDHCSTEER Build the deterministic Reeds-Shepp geometry into a MATLAB MEX.
% BuildBIAGT uses the compiler selected by mex -setup C++.
% BuildBIAGT('.../zig.exe') supports portable Zig 0.13 on Windows.
% All binaries and compiler caches stay outside the source tree.
if nargin<1,zigExecutable='';end
root=fileparts(mfilename('fullpath'));native=fullfile(root,'native');
folder=getenv('COMMONPARKING_BIAGT_DIR');
if isempty(folder),folder=fullfile(tempdir,'CommonParking','biagt',computer('arch'));end
if ~isfolder(folder),mkdir(folder);end
shared=fullfile(fileparts(which('BuildHCSteer')),'native','steering_functions');
sources={fullfile(native,'biagt_rs_mex.cpp'),fullfile(native,'steering_functions','src','reeds_shepp_state_space.cpp'),fullfile(shared,'src','utilities','utilities.cpp')};
include=fullfile(native,'steering_functions','include');sharedInclude=fullfile(shared,'include');
if isempty(zigExecutable)
    mex('-R2017b',['-I',include],['-I',sharedInclude],sources{:},'-outdir',folder,'-output','biagt_rs_mex');
else
    assert(ispc&&isfile(zigExecutable),'Portable Zig build requires Windows and an existing executable.');
    setenv('ZIG_GLOBAL_CACHE_DIR',fullfile(folder,'zig-cache'));setenv('ZIG_LOCAL_CACHE_DIR',fullfile(folder,'zig-local-cache'));
    libraries=fullfile(matlabroot,'extern','lib','win64','microsoft');
    args=[{zigExecutable,'c++','-shared','-O2','-std=c++14','-static','-Wno-dll-attribute-on-redeclaration', ...
        '-I',fullfile(matlabroot,'extern','include'),'-I',include,'-I',sharedInclude},sources, ...
        {fullfile(libraries,'libmex.lib'),fullfile(libraries,'libmx.lib'),'-o',fullfile(folder,['biagt_rs_mex.',mexext])}];
    % No shell interpretation: paths are passed directly to the child process.
    info=System.Diagnostics.ProcessStartInfo;info.FileName=zigExecutable;
    info.Arguments=strjoin(cellfun(@(s)['"',strrep(s,'"',''), '"'],args(2:end),'UniformOutput',false),' ');
    info.UseShellExecute=false;info.CreateNoWindow=true;
    process=System.Diagnostics.Process;process.StartInfo=info;process.Start();process.WaitForExit();
    code=process.ExitCode;process.Dispose();assert(code==0,'BIAGT RS compilation failed.');
end
addpath(folder);assert(exist('biagt_rs_mex','file')==3,'Compiled MATLAB module was not found.');
fprintf('BIAGT RS MATLAB module: %s\n',folder);
end
