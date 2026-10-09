function folder = BuildCCSteer(zigExecutable)
%BUILDCCSTEER Build the deterministic authors' geometry into a MATLAB MEX.
% BuildCCSteer uses the compiler selected by mex -setup C++.
% BuildCCSteer('.../zig.exe') supports portable Zig 0.13 on Windows.
% All binaries and compiler caches stay outside the source tree.
if nargin<1,zigExecutable='';end
root=fileparts(mfilename('fullpath'));native=fullfile(root,'native');
folder=getenv('COMMONPARKING_CC_DIR');
if isempty(folder),folder=fullfile(tempdir,'CommonParking','cc-steer',computer('arch'));end
if ~isfolder(folder),mkdir(folder);end
files=dir(fullfile(native,'steering_functions','src','**','*.cpp'));
sources=[{fullfile(native,'cc_steer_mex.cpp')},arrayfun(@(x)fullfile(x.folder,x.name),files,'UniformOutput',false)'];
include=fullfile(native,'steering_functions','include');
if isempty(zigExecutable)
    mex('-R2017b',['-I',include],['-I',native],sources{:},'-outdir',folder,'-output','cc_steer_mex');
else
    assert(ispc&&isfile(zigExecutable),'Portable Zig build requires Windows and an existing executable.');
    setenv('ZIG_GLOBAL_CACHE_DIR',fullfile(folder,'zig-cache'));setenv('ZIG_LOCAL_CACHE_DIR',fullfile(folder,'zig-local-cache'));
    libraries=fullfile(matlabroot,'extern','lib','win64','microsoft');
    args=[{zigExecutable,'c++','-shared','-O2','-std=c++14','-static','-Wno-dll-attribute-on-redeclaration', ...
        '-I',fullfile(matlabroot,'extern','include'),'-I',include,'-I',native},sources, ...
        {fullfile(libraries,'libmex.lib'),fullfile(libraries,'libmx.lib'),'-o',fullfile(folder,['cc_steer_mex.',mexext])}];
    % No shell interpretation: paths are passed directly to the child process.
    info=System.Diagnostics.ProcessStartInfo;info.FileName=zigExecutable;
    info.Arguments=strjoin(cellfun(@(s)['"',strrep(s,'"',''), '"'],args(2:end),'UniformOutput',false),' ');
    info.UseShellExecute=false;info.CreateNoWindow=true;
    process=System.Diagnostics.Process;process.StartInfo=info;process.Start();process.WaitForExit();
    code=process.ExitCode;process.Dispose();assert(code==0,'CC-Steer compilation failed.');
end
addpath(folder);assert(exist('cc_steer_mex','file')==3,'Compiled MATLAB module was not found.');
fprintf('CC-Steer MATLAB module: %s\n',folder);
end
