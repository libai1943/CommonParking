function root = SetupCommonParking()
%SETUPCOMMONPARKING Add source directories without changing the current folder.
root = fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'common'),fullfile(root,'evaluation'));
folders = dir(fullfile(root,'planners'));
for k = 1:numel(folders)
    if folders(k).isdir && ~startsWith(folders(k).name,'.')
        addpath(fullfile(folders(k).folder,folders(k).name));
    end
end
end
