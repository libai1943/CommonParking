function EnsureNative()
if exist('lattice_graph_mex','file')==3,return;end
folder=getenv('COMMONPARKING_LATTICE_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','lattice-graph',computer('arch'));end
if isfile(fullfile(folder,['lattice_graph_mex.',mexext])),addpath(folder);end
assert(exist('lattice_graph_mex','file')==3,'Run BuildLatticeGraph once with a configured MATLAB C++ compiler.');
end
