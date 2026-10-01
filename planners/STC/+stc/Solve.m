function [trajectory,solver] = Solve(c,initial,boxes,discs,options)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();
root=fileparts(fileparts(mfilename('fullpath')));copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));
N=numel(initial.t);v=c.vehicle;f=fopen(fullfile(folder,'data.dat'),'w');closeFile=onCleanup(@()fclose(f));
names={'N','D','lw','vmax','amax','phimax','wmax','max_time'};
values=[N discs.count v.lw v.vmax v.amax v.phimax v.wmax options.maxTime];
for j=1:numel(names),fprintf(f,'param %s := %.17g;\n',names{j},values(j));end
t=c.task;boundary=[t.x0 t.y0 t.theta0 t.xf t.yf initial.theta(end)];
fprintf(f,'param boundary :=\n');fprintf(f,'%d %.17g\n',[(1:6)' boundary']');fprintf(f,';\n');
fprintf(f,'param offset: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:discs.count)' discs.offsets]');fprintf(f,';\n');
fprintf(f,'param box :=\n');
for j=1:discs.count
    for side=1:4,fprintf(f,['%d ',num2str(j),' ',num2str(side),' %.17g\n'],[(1:N)' boxes(:,j,side)]');end
end
fprintf(f,';\n');clear closeFile;
f=fopen(fullfile(folder,'initial.run'),'w');closeFile=onCleanup(@()fclose(f));
fprintf(f,'let tf := %.17g;\n',min(initial.t(end),options.maxTime));
fields={'x','y','theta','v','phi','a','omega'};variables={'x','y','theta','v','phi','a','w'};
for j=1:numel(fields),fprintf(f,['let ',variables{j},'[%d] := %.17g;\n'],[(1:N)' initial.(fields{j})]');end
clear closeFile;
f=fopen(fullfile(folder,'solve.run'),'w');closeFile=onCleanup(@()fclose(f));
fprintf(f,'reset;\nmodel model.mod;\ndata data.dat;\ninclude initial.run;\n');
fprintf(f,'option solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=1e-7 acceptable_tol=1e-7 max_iter=%d max_cpu_time=%.9g print_level=3";\n',options.maxIterations,options.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf {i in 1..N} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",tf*(i-1)/(N-1),x[i],y[i],theta[i],v[i],phi[i],a[i],w[i] > "candidate.txt";\n');
clear closeFile;
old=pwd;restore=onCleanup(@()cd(old));cd(folder);[process,log]=system(['"',runtime.ampl,'" solve.run']);
f=fopen('solver.log','w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message','','working_directory',folder);
trajectory=[];
if isfile('status.txt')
    lines=splitlines(string(fileread('status.txt')));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
    solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
else,solver.message=log;end
if isfile('candidate.txt')
    z=readmatrix('candidate.txt','FileType','text');
    if isequal(size(z),[N 8])&&isreal(z)&&all(isfinite(z),'all')&&all(diff(z(:,1))>0)
        trajectory=struct('t',z(:,1),'x',z(:,2),'y',z(:,3),'theta',z(:,4),'v',z(:,5),'phi',z(:,6),'a',z(:,7),'omega',z(:,8));
    else,solver.success=false;solver.message='Invalid candidate output.';end
else
    solver.success=false;solver.message='Solver did not write a candidate output.';
end
end
