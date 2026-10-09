function [result,metrics,evaluationStatus] = LoadPublishedResult(plannerName,caseId)
% Load a released result, including losslessly chunked large MAT artifacts.
validateattributes(caseId,{'numeric'},{'scalar','integer','>=',1,'<=',12});
plannerName=char(string(plannerName));
assert(~isempty(regexp(plannerName,'^[A-Za-z][A-Za-z0-9_]*$','once')),'Invalid planner name.');
folder=fullfile(fileparts(mfilename('fullpath')),'results',plannerName);
stem=sprintf('Case%02d',caseId);filename=fullfile(folder,[stem '.mat']);
if isfile(filename)
    data=load(filename);
else
    manifest=jsondecode(fileread(fullfile(folder,[stem '.storage.json'])));
    assert(strcmp(manifest.format,'mat-v7-chunks-v1'),'Unsupported result storage.');
    bytes=uint8([]);
    for k=1:numel(manifest.parts)
        name=manifest.parts{k};
        assert(~isempty(regexp(name,'^[A-Za-z0-9_.-]+$','once')) && ~contains(name,'..'),'Invalid chunk name.');
        f=fopen(fullfile(folder,name),'rb');assert(f>=0,'Missing result chunk.');
        cleanup=onCleanup(@()fclose(f));part=fread(f,Inf,'*uint8');clear cleanup;
        bytes=[bytes;part]; %#ok<AGROW>
    end
    assert(numel(bytes)==manifest.bytes,'Result byte count mismatch.');
    digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
    actual=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
    assert(strcmp(actual,manifest.sha256),'Result checksum mismatch.');
    filename=[tempname '.mat'];cleanup=onCleanup(@()removeTemporary(filename));
    f=fopen(filename,'wb');assert(f>=0,'Cannot create temporary result file.');
    closer=onCleanup(@()fclose(f));assert(fwrite(f,bytes,'uint8')==numel(bytes));clear closer;
    data=load(filename);clear cleanup;
end
result=data.result;metrics=data.metrics;evaluationStatus=data.evaluationStatus;
end

function removeTemporary(filename)
if isfile(filename),delete(filename);end
end
