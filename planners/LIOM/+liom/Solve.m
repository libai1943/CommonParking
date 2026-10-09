function [trajectory,solver] = Solve(c,initial,boxes,discs,options)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();
root=fileparts(fileparts(mfilename('fullpath')));copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));
N=numel(initial.t);v=c.vehicle;f=fopen(fullfile(folder,'data.dat'),'w');closeFile=onCleanup(@()fclose(f));
names={'N','D','lw','vmax','amax','phimax','wmax','max_time','weight_energy','weight_penalty'};
values=[N discs.count v.lw v.vmax v.amax v.phimax v.wmax options.maxTime options.energyWeight options.penaltyWeight];
for j=1:numel(names),fprintf(f,'param %s := %.17g;\n',names{j},values(j));end
t=c.task;boundary=[t.x0 t.y0 t.theta0 t.xf t.yf t.thetaf];
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
o=discs.offsets;
if isfield(initial,'cx')&&size(initial.cx,2)==discs.count
    cx=initial.cx;cy=initial.cy;
else
    cx=initial.x+cos(initial.theta)*o(:,1)'-sin(initial.theta)*o(:,2)';
    cy=initial.y+sin(initial.theta)*o(:,1)'+cos(initial.theta)*o(:,2)';
end
cx=min(max(cx,boxes(:,:,1)),boxes(:,:,2));cy=min(max(cy,boxes(:,:,3)),boxes(:,:,4));
for j=1:discs.count
    fprintf(f,['let cx[%d,',num2str(j),'] := %.17g;\n'],[(1:N)' cx(:,j)]');
    fprintf(f,['let cy[%d,',num2str(j),'] := %.17g;\n'],[(1:N)' cy(:,j)]');
end
clear closeFile;
f=fopen(fullfile(folder,'solve.run'),'w');closeFile=onCleanup(@()fclose(f));
fprintf(f,'reset;\nmodel model.mod;\ndata data.dat;\ninclude initial.run;\n');
fprintf(f,'option solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=1e-8 acceptable_tol=1e-8 max_iter=%d max_cpu_time=%.9g print_level=3 nlp_scaling_method=none mu_strategy=monotone mu_init=1e-12 bound_push=1e-12 bound_frac=1e-12 bound_mult_init_val=1e-12 linear_solver=ma97";\n',options.maxIterations,options.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g %%.17g %%.17g %%.17g %%.17g\\n",infeasibility,dynamic_penalty,geometry_penalty,heading_penalty,nominal_cost > "costs.txt";\n');
fprintf(f,'printf {i in 1..N} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",tf*(i-1)/(N-1),x[i],y[i],theta[i],v[i],phi[i],a[i],w[i] > "candidate.txt";\n');
fprintf(f,'printf {j in 1..D,i in 1..N} "%%.17g %%.17g\\n",cx[i,j],cy[i,j] > "centres.txt";\n');
clear closeFile;
old=pwd;restore=onCleanup(@()cd(old));cd(folder);
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',options.maxWallSeconds);
f=fopen('solver.log','w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message','', ...
    'infeasibility',inf,'working_directory',folder);trajectory=[];
if isfile('status.txt')
    lines=splitlines(string(fileread('status.txt')));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
    solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
else,solver.message=log;end
if isfile('costs.txt')
    cost=readmatrix('costs.txt','FileType','text');
    if numel(cost)==5&&isreal(cost)&&all(isfinite(cost))
        solver.infeasibility=cost(1);solver.dynamic_penalty=cost(2);solver.geometry_penalty=cost(3);solver.heading_penalty=cost(4);solver.nominal_cost=cost(5);
    else,solver.success=false;end
end
if isfile('candidate.txt')&&isfile('centres.txt')
    z=readmatrix('candidate.txt','FileType','text');centres=readmatrix('centres.txt','FileType','text');
    if isequal(size(z),[N 8])&&isreal(z)&&all(isfinite(z),'all')&&all(diff(z(:,1))>0) ...
            &&isequal(size(centres),[N*discs.count 2])&&isreal(centres)&&all(isfinite(centres),'all')
        trajectory=struct('t',z(:,1),'x',z(:,2),'y',z(:,3),'theta',z(:,4),'v',z(:,5),'phi',z(:,6),'a',z(:,7),'omega',z(:,8), ...
            'cx',reshape(centres(:,1),N,discs.count),'cy',reshape(centres(:,2),N,discs.count));
    else,solver.success=false;solver.message='Invalid candidate output.';end
else
    solver.success=false;solver.message=[solver.message,' Solver did not write complete candidate output.'];
end
check=liom.CheckCandidate(c,trajectory,boxes,discs,options);
solver.candidate_check=check;
costMatches=check.valid&&abs(check.infeasibility-solver.infeasibility)<=1e-10*max(1,check.infeasibility);
solver.success=solver.success&&costMatches;
% A time/iteration limit is an inexact INNER solve, not a failed LIOM outer
% iteration. Preserve its native flag; only a valid bounded iterate continues.
solver.continue_outer=costMatches&&process==0&&(solver.success||ismember(solver.solve_result_num,[400 401]));
tokens=regexp(log,'Number of Iterations[. ]*:\s*(\d+)','tokens','once');
solver.iterations=NaN;if ~isempty(tokens),solver.iterations=str2double(tokens{1});end
end
