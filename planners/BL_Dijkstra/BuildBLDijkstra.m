function folder=BuildBLDijkstra(zigExecutable)
%BUILDBLDIJKSTRA Build the minimum-reversal search core outside the repository.
if nargin<1,zigExecutable='';end
root=fileparts(mfilename('fullpath'));source=fullfile(root,'native','bl_dijkstra_mex.cpp');
folder=getenv('COMMONPARKING_BL_DIJKSTRA_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','bl-dijkstra',computer('arch'));end
if ~isfolder(folder),mkdir(folder);end
if isempty(zigExecutable)
 mex('-R2017b',source,'-outdir',folder,'-output','bl_dijkstra_mex');
else
 assert(ispc&&isfile(zigExecutable),'Portable Zig build requires Windows and an existing executable.');
 setenv('ZIG_GLOBAL_CACHE_DIR',fullfile(folder,'zig-cache'));setenv('ZIG_LOCAL_CACHE_DIR',fullfile(folder,'zig-local-cache'));
 libraries=fullfile(matlabroot,'extern','lib','win64','microsoft');
 args={'c++','-shared','-O2','-std=c++14','-static','-Wno-dll-attribute-on-redeclaration','-I',fullfile(matlabroot,'extern','include'),source,fullfile(libraries,'libmex.lib'),fullfile(libraries,'libmx.lib'),'-o',fullfile(folder,['bl_dijkstra_mex.',mexext])};
 info=System.Diagnostics.ProcessStartInfo;info.FileName=zigExecutable;
 info.Arguments=strjoin(cellfun(@(s)['"',strrep(s,'"',''), '"'],args,'UniformOutput',false),' ');info.UseShellExecute=false;info.CreateNoWindow=true;
 process=System.Diagnostics.Process;process.StartInfo=info;process.Start();process.WaitForExit();code=process.ExitCode;process.Dispose();assert(code==0,'BL_Dijkstra compilation failed.');
end
addpath(folder);assert(exist('bl_dijkstra_mex','file')==3);fprintf('BL_Dijkstra MATLAB module: %s\n',folder);
end
