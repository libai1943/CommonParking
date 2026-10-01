function folder = RuntimeDirectory()
% Solver jobs never modify the repository or the caller's working directory.
root=getenv('COMMONPARKING_WORK');
if isempty(root),root=fullfile(tempdir,'CommonParking');end
if ~exist(root,'dir'),mkdir(root);end
folder=tempname(root);mkdir(folder);
end
