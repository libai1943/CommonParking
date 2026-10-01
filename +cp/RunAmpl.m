function [exitCode,log] = RunAmpl(runtime,folder,script,maxWallSeconds)
% Bound a Windows AMPL process tree, including time in a single factorization.
% Threads are fixed for reproducible resource use; no global MATLAB env change.
assert(ispc,'This process wrapper currently requires Windows .NET.');
info=System.Diagnostics.ProcessStartInfo();info.FileName=runtime.ampl;
info.Arguments=['"',script,'"'];info.WorkingDirectory=folder;
info.UseShellExecute=false;info.CreateNoWindow=true;
info.RedirectStandardOutput=true;info.RedirectStandardError=true;
environment=info.EnvironmentVariables;
for key={'OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS'}
    environment.Remove(key{1});environment.Add(key{1},'1');
end
process=System.Diagnostics.Process();process.StartInfo=info;process.Start();
output=process.StandardOutput.ReadToEndAsync();errors=process.StandardError.ReadToEndAsync();
finished=process.WaitForExit(int32(ceil(maxWallSeconds*1000)));
if finished
    exitCode=double(process.ExitCode);
else
    % The PID is obtained only from this newly created, owned process.
    system(sprintf('taskkill /PID %d /T /F > NUL 2>&1',process.Id));
    process.WaitForExit();exitCode=124;
end
log=[char(output.Result),newline,char(errors.Result)];
if ~finished,log=[log,newline,sprintf('CommonParking: wall-clock limit %.3f s exceeded.',maxWallSeconds)];end
process.Dispose();
end
