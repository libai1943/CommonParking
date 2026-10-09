function folder=BuildGraphBellman(zigExecutable)
%BUILDGRAPHBELLMAN Build the switched-system Bellman core outside the repository.
if nargin<1,zigExecutable='';end
root=fileparts(mfilename('fullpath'));source=fullfile(root,'native','graph_bellman_mex.cpp');
folder=getenv('COMMONPARKING_GRAPH_BELLMAN_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','graph-bellman',computer('arch'));end
if ~isfolder(folder),mkdir(folder);end
if isempty(zigExecutable)
 mex('-R2017b',source,'-outdir',folder,'-output','graph_bellman_mex');
else
 assert(ispc&&isfile(zigExecutable),'Portable Zig build requires Windows and an existing executable.');
 setenv('ZIG_GLOBAL_CACHE_DIR',fullfile(folder,'zig-cache'));setenv('ZIG_LOCAL_CACHE_DIR',fullfile(folder,'zig-local-cache'));
 libraries=fullfile(matlabroot,'extern','lib','win64','microsoft');
 args={'c++','-shared','-O2','-std=c++14','-static','-Wno-dll-attribute-on-redeclaration','-I',fullfile(matlabroot,'extern','include'),source,fullfile(libraries,'libmex.lib'),fullfile(libraries,'libmx.lib'),'-o',fullfile(folder,['graph_bellman_mex.',mexext])};
 info=System.Diagnostics.ProcessStartInfo;info.FileName=zigExecutable;
 info.Arguments=strjoin(cellfun(@(s)['"',strrep(s,'"',''), '"'],args,'UniformOutput',false),' ');info.UseShellExecute=false;info.CreateNoWindow=true;
 process=System.Diagnostics.Process;process.StartInfo=info;process.Start();process.WaitForExit();code=process.ExitCode;process.Dispose();assert(code==0,'GraphBellman compilation failed.');
end
addpath(folder);assert(exist('graph_bellman_mex','file')==3);fprintf('GraphBellman MATLAB module: %s\n',folder);
end
