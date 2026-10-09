function root = SetupCommonParking()
%SETUPCOMMONPARKING Add source directories without changing the current folder.
root = fileparts(mfilename('fullpath'));
wanted = {root,fullfile(root,'common'),fullfile(root,'evaluation')};
folders = dir(fullfile(root,'planners'));
for k = 1:numel(folders)
    if folders(k).isdir && ~startsWith(folders(k).name,'.')
        wanted{end+1} = fullfile(folders(k).folder,folders(k).name); %#ok<AGROW>
    end
end
% Discover newly installed planners on every call, but do not repeatedly
% invalidate MATLAB's path cache for directories that are already present.
present = strsplit(path,pathsep);
normalise = @(p) lower(strrep(string(p),'\','/'));
missing = ~ismember(normalise(wanted),normalise(present));
if any(missing),addpath(wanted{missing});end
end
