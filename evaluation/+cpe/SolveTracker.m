function [native,solver] = SolveTracker(ref,c,cfg,nodes)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();
source=fullfile(fileparts(fileparts(mfilename('fullpath'))),'tracker.mod');
copyfile(source,fullfile(folder,'tracker.mod'));
q=cpe.ReferenceAt(ref,linspace(0,ref.tf,nodes)',c.vehicle);
% Remove absolute translation for numerical conditioning; keep task geometry.
origin=[c.task.x0 c.task.y0];q.x=q.x-origin(1);q.y=q.y-origin(2);
e=cfg.evaluation;v=c.vehicle;relax=1+e.limit_relaxation;
f=fopen(fullfile(folder,'problem.dat'),'w');closeFile=onCleanup(@()fclose(f));
names={'N','Tref','L','V','A','P','W','Tmin','Tmax','time_weight','smooth_weight'};
values=[nodes ref.tf v.lw v.vmax*relax v.amax*relax v.phimax*relax v.wmax*relax ...
    ref.tf*e.time_scale_bounds e.time_regularization e.control_smoothing];
for j=1:numel(names),fprintf(f,'param %s := %.17g;\n',names{j},values(j));end
fprintf(f,'param q0 := 1 0 2 0 3 %.17g;\n',c.task.theta0);
fprintf(f,'param ref: 1 2 3 4 5 :=\n');
fprintf(f,'%d %.17g %.17g %.17g %.17g %.17g\n',[(1:nodes)' q.x q.y q.theta q.v q.phi]');
fprintf(f,';\n');clear closeFile;
f=fopen(fullfile(folder,'initial.run'),'w');closeFile=onCleanup(@()fclose(f));
fprintf(f,'let tf := %.17g;\n',ref.tf);
q.a=gradient(q.v,q.t);q.omega=gradient(q.phi,q.t);
q.v([1 end])=0;q.phi([1 end])=0;
names={'x','y','theta','v','phi','a','w'};fields={'x','y','theta','v','phi','a','omega'};
for j=1:numel(names)
    z=q.(fields{j});
    if strcmp(names{j},'a'),z=max(-v.amax,min(v.amax,z));end
    if strcmp(names{j},'w'),z=max(-v.wmax,min(v.wmax,z));end
    fprintf(f,['let ',names{j},'[%d] := %.17g;\n'],[(1:nodes)' z]');
end
clear closeFile;
f=fopen(fullfile(folder,'solve.run'),'w');closeFile=onCleanup(@()fclose(f));
fprintf(f,'reset;\nmodel tracker.mod;\ndata problem.dat;\ninclude initial.run;\n');
fprintf(f,'option solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.12g acceptable_tol=%.12g nlp_scaling_method=none max_iter=%d max_cpu_time=%.9g print_level=3";\n',e.tracker_tolerance,e.tracker_tolerance,e.max_iterations,e.max_cpu_seconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf {i in 1..N} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",tf*(i-1)/(N-1),x[i],y[i],theta[i],v[i],phi[i],a[i],w[i] > "candidate.txt";\n');
clear closeFile;
previous=pwd;restore=onCleanup(@()cd(previous));cd(folder);
[process,log]=system(['"',runtime.ampl,'" solve.run']);
f=fopen('solver.log','w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN, ...
    'message','','node_count',nodes,'working_directory',folder);
native=[];
if isfile('status.txt')
    lines=splitlines(string(fileread('status.txt')));solver.solve_result_num=str2double(lines(1));
    solver.message=char(join(lines(2:end),newline));
    solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
else
    solver.message=log;
end
if isfile('candidate.txt')
    z=readmatrix('candidate.txt','FileType','text');
    if isequal(size(z),[nodes 8])&&isreal(z)&&all(isfinite(z),'all')&&all(diff(z(:,1))>0)
        native=struct('t',z(:,1),'x',z(:,2)+origin(1),'y',z(:,3)+origin(2), ...
            'theta',z(:,4),'v',z(:,5),'phi',z(:,6),'a',z(:,7),'omega',z(:,8));
    else
        solver.success=false;solver.message='Tracking solver returned malformed data.';
    end
end
end
