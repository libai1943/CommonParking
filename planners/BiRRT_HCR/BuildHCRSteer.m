function folder = BuildHCRSteer(zigExecutable)
%BUILDHCRSTEER Build cubic-spiral HCR geometry into a MATLAB MEX.
% BuildHCRSteer uses the compiler selected by mex -setup C++.
% BuildHCRSteer('.../zig.exe') supports portable Zig 0.13 on Windows.
% All binaries and compiler caches stay outside the source tree.
if nargin<1,zigExecutable='';end
root=fileparts(mfilename('fullpath'));native=fullfile(root,'native');
folder=getenv('COMMONPARKING_HCR_DIR');
if isempty(folder),folder=fullfile(tempdir,'CommonParking','hcr-steer',computer('arch'));end
if ~isfolder(folder),mkdir(folder);end
files=dir(fullfile(native,'steering_functions','src','**','*.cpp'));
sources=[{fullfile(native,'hcr_steer_mex.cpp')},arrayfun(@(x)fullfile(x.folder,x.name),files,'UniformOutput',false)'];
include=fullfile(native,'steering_functions','include');
if isempty(zigExecutable)
    mex('-R2017b',['-I',include],['-I',native],sources{:},'-outdir',folder,'-output','hcr_steer_mex');
else
    assert(ispc&&isfile(zigExecutable),'Portable Zig build requires Windows and an existing executable.');
    setenv('ZIG_GLOBAL_CACHE_DIR',fullfile(folder,'zig-cache'));setenv('ZIG_LOCAL_CACHE_DIR',fullfile(folder,'zig-local-cache'));
    libraries=fullfile(matlabroot,'extern','lib','win64','microsoft');
    args=[{zigExecutable,'c++','-shared','-O2','-std=c++14','-static','-Wno-dll-attribute-on-redeclaration', ...
        '-I',fullfile(matlabroot,'extern','include'),'-I',include,'-I',native},sources, ...
        {fullfile(libraries,'libmex.lib'),fullfile(libraries,'libmx.lib'),'-o',fullfile(folder,['hcr_steer_mex.',mexext])}];
    % No shell interpretation: paths are passed directly to the child process.
    info=System.Diagnostics.ProcessStartInfo;info.FileName=zigExecutable;
    info.Arguments=strjoin(cellfun(@(s)['"',strrep(s,'"',''), '"'],args(2:end),'UniformOutput',false),' ');
    info.UseShellExecute=false;info.CreateNoWindow=true;
    process=System.Diagnostics.Process;process.StartInfo=info;process.Start();process.WaitForExit();
    code=process.ExitCode;process.Dispose();assert(code==0,'HCR-Steer compilation failed.');
end
addpath(folder);assert(exist('hcr_steer_mex','file')==3,'Compiled MATLAB module was not found.');
fprintf('HCR-Steer MATLAB module: %s\n',folder);
end
