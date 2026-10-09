function folder=BuildHJBA(zigExecutable)
%BUILDHJBA Optional acceleration of the unchanged MATLAB HJ stencil.
if nargin<1,zigExecutable='';end
root=fileparts(mfilename('fullpath'));source=fullfile(root,'native','hjba_hj_mex.cpp');
folder=getenv('COMMONPARKING_HJBA_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','hjba',computer('arch'));end
if ~isfolder(folder),mkdir(folder);end
if isempty(zigExecutable)
 mex('-R2017b',source,'-outdir',folder,'-output','hjba_hj_mex');
else
 assert(ispc&&isfile(zigExecutable),'Portable Zig build requires Windows and an existing executable.');
 setenv('ZIG_GLOBAL_CACHE_DIR',fullfile(folder,'zig-cache'));setenv('ZIG_LOCAL_CACHE_DIR',fullfile(folder,'zig-local-cache'));
 libraries=fullfile(matlabroot,'extern','lib','win64','microsoft');
 args={'c++','-shared','-O2','-std=c++14','-static','-Wno-dll-attribute-on-redeclaration','-I',fullfile(matlabroot,'extern','include'),source,fullfile(libraries,'libmex.lib'),fullfile(libraries,'libmx.lib'),'-o',fullfile(folder,['hjba_hj_mex.',mexext])};
 info=System.Diagnostics.ProcessStartInfo;info.FileName=zigExecutable;info.Arguments=strjoin(cellfun(@(s)['"',strrep(s,'"',''),'"'],args,'UniformOutput',false),' ');info.UseShellExecute=false;info.CreateNoWindow=true;
 process=System.Diagnostics.Process;process.StartInfo=info;process.Start();process.WaitForExit();code=process.ExitCode;process.Dispose();assert(code==0,'HJBA compilation failed.');
end
addpath(folder);assert(exist('hjba_hj_mex','file')==3);fprintf('HJBA MATLAB module: %s\n',folder);
end
