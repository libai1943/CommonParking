function [trajectory,solver] = Solve(c,initial,polygons,lambda,mu,options)
% MATLAB/AMPL port of the paper equations; dual initialization is solved first.
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));
copyfile(fullfile(root,'model.mod'),folder);copyfile(fullfile(root,'dual.mod'),folder);
N=numel(initial.t);M=polygons.count;counts=cellfun(@(a)size(a,1),polygons.A);E=sum(counts);last=cumsum(counts);first=last-counts+1;
A=vertcat(polygons.A{:});b=vertcat(polygons.b{:});v=c.vehicle;g=[v.lw+v.lf v.lr v.lb/2 v.lb/2];
f=fopen(fullfile(folder,'shapes.dat'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'param N:=%d;\nparam M:=%d;\nparam E:=%d;\n',N,M,E);
fprintf(f,'param: first last :=\n');fprintf(f,'%d %d %d\n',[(1:M)' first last]');fprintf(f,';\n');
fprintf(f,'param A: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:E)' A]');fprintf(f,';\n');
writeVector(f,'b',b);writeVector(f,'g',g);clear closer;
f=fopen(fullfile(folder,'poses.dat'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'param pose: 1 2 3 :=\n');fprintf(f,'%d %.17g %.17g %.17g\n',[(1:N)' initial.x initial.y initial.theta]');fprintf(f,';\n');clear closer;
f=fopen(fullfile(folder,'dual_initial.run'),'w');closer=onCleanup(@()fclose(f));
for j=1:E,fprintf(f,['let lambda[%d,',num2str(j),']:=%.17g;\n'],[(1:N)' lambda(:,j)]');end
for j=1:M,for k=1:4,fprintf(f,['let mu[%d,',num2str(j),',',num2str(k),']:=%.17g;\n'],[(1:N)' mu(:,j,k)]');end,end
clear closer;
f=fopen(fullfile(folder,'dual_solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model dual.mod;data shapes.dat;data poses.dat;include dual_initial.run;\n');
writeOptions(f,runtime,options);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "dual_status.txt";\n');
fprintf(f,'printf {i in 1..N,k in 1..E} "let lambda[%%d,%%d]:=%%.17g;\\n",i,k,lambda[i,k] > "dual_solution.run";\n');
fprintf(f,'printf {i in 1..N,j in 1..M,k in 1..4} "let mu[%%d,%%d,%%d]:=%%.17g;\\n",i,j,k,mu[i,j,k] >> "dual_solution.run";\n');clear closer;
old=pwd;restore=onCleanup(@()cd(old));cd(folder);
[process,log]=cp.RunAmpl(runtime,folder,'dual_solve.run',2*options.maxCpuSeconds+30);writeLog('dual.log',log);
aux=readStatus('dual_status.txt',process,log);solver=aux;solver.dual_initialization=aux;solver.working_directory=folder;trajectory=[];
if ~aux.success,solver.message=['Distance-dual initialization failed: ',aux.message];return;end
f=fopen('settings.dat','w');closer=onCleanup(@()fclose(f));
names={'lw','vmax','amax','phimax','wmax','dmin','tref','scale_min','scale_max','time_weight'};
values=[v.lw v.vmax v.amax v.phimax v.wmax options.clearance initial.t(end) options.timeScale options.timeWeightPerSecond];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
t=c.task;writeVector(f,'boundary',[t.x0 t.y0 t.theta0 0 t.xf t.yf initial.theta(end) 0]);
writeVector(f,'q_control',options.controlWeights);writeVector(f,'q_rate',options.rateWeights);clear closer;
f=fopen('initial.run','w');closer=onCleanup(@()fclose(f));fprintf(f,'let tf:=%.17g;\n',initial.t(end));
initial.phi([1 end])=0;initial.omega=[diff(initial.phi)./diff(initial.t);0];
for key={'x','y','theta','v','phi'}
    name=key{1};fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(1:N)' initial.(name)]');
end
for key={'omega','a'}
    name=key{1};z=initial.(name);fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(1:N-1)' z(1:N-1)]');
end
clear closer;
for attempt=1:options.maxRestarts+1
    f=fopen('solve.run','w');closer=onCleanup(@()fclose(f));
    fprintf(f,'reset;model model.mod;data shapes.dat;data settings.dat;include initial.run;include dual_solution.run;\n');
    if attempt>1,fprintf(f,'include restart.run;\n');end
    writeOptions(f,runtime,options);fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
    fprintf(f,'printf {i in 1..N} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",tf*(i-1)/(N-1),x[i],y[i],theta[i],v[i],phi[i] > "states.txt";\n');
    fprintf(f,'printf {i in 1..N-1} "%%.17g %%.17g\\n",omega[i],a[i] > "controls.txt";\n');
    fprintf(f,'printf {i in 1..N,k in 1..E} "%%.17g\\n",lambda[i,k] > "lambda.txt";\n');
    fprintf(f,'printf {i in 1..N,j in 1..M,k in 1..4} "%%.17g\\n",mu[i,j,k] > "mu.txt";\n');
    fprintf(f,'printf "%%.17g\\n",cost > "cost.txt";\n');
    fprintf(f,'printf "let tf:=%%.17g;\\n",tf > "restart.run";\n');
    for key={'x','y','theta','v','phi','a','omega'}
        name=key{1};count=N;if ismember(name,{'omega','a'}),count=N-1;end
        fprintf(f,['printf {i in 1..',num2str(count),'} "let ',name,'[%%d]:=%%.17g;\\n",i,',name,'[i] >> "restart.run";\n']);
    end
    fprintf(f,'printf {i in 1..N,k in 1..E} "let lambda[%%d,%%d]:=%%.17g;\\n",i,k,lambda[i,k] >> "restart.run";\n');
    fprintf(f,'printf {i in 1..N,j in 1..M,k in 1..4} "let mu[%%d,%%d,%%d]:=%%.17g;\\n",i,j,k,mu[i,j,k] >> "restart.run";\n');clear closer;
    [process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*options.maxCpuSeconds+30);writeLog(sprintf('solver_%d.log',attempt),log);
    current=readStatus('status.txt',process,log);solver=current;solver.dual_initialization=aux;solver.working_directory=folder;solver.attempts=attempt;
    if solver.success,break;end
end
if isfile('states.txt')&&isfile('controls.txt')
    z=readmatrix('states.txt','FileType','text');u=readmatrix('controls.txt','FileType','text');
    if isequal(size(z),[N 6])&&isequal(size(u),[N-1 2])&&isreal(z)&&isreal(u)&&all(isfinite(z),'all')&&all(isfinite(u),'all')&&all(diff(z(:,1))>0)
        phi=z(:,6);a=[u(:,2);0];omega=[u(:,1);0];
        trajectory=struct('t',z(:,1),'x',z(:,2),'y',z(:,3),'theta',z(:,4),'v',z(:,5),'phi',phi,'a',a,'omega',omega);
        if all(cellfun(@isfile,{'lambda.txt','mu.txt','cost.txt'}))
            l=readmatrix('lambda.txt','FileType','text');m=readmatrix('mu.txt','FileType','text');cost=readmatrix('cost.txt','FileType','text');
            if numel(l)==N*E&&numel(m)==N*M*4&&isscalar(cost)&&isreal(l)&&isreal(m)&&isreal(cost)&&all(isfinite([l(:);m(:);cost]))
                lambda=reshape(l,E,N)';mu=permute(reshape(m,4,M,N),[3 2 1]);
                solver.check=hobca.Check(c,trajectory,lambda,mu,polygons,initial.t(end),options);
                solver.objective=cost;solver.objective_error=abs(cost-solver.check.objective);
                solver.success=solver.success&&solver.check.success&&solver.objective_error<1e-6;
            else,solver.success=false;solver.message='Invalid native dual/cost output.';end
        else,solver.success=false;solver.message='Native dual/cost output missing.';end

    else,solver.success=false;solver.message='Invalid solver output.';end
else,solver.success=false;solver.message='Solver output missing.';end
end
function writeVector(f,name,x)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(x))' x(:)]');fprintf(f,';\n');
end
function writeOptions(f,runtime,options)
fprintf(f,'option solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=1e-9 acceptable_tol=1e-9 max_iter=%d max_cpu_time=%.9g print_level=3 mu_strategy=adaptive linear_solver=ma97";\n',options.maxIterations,options.maxCpuSeconds);
end
function r=readStatus(file,process,log)
r=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log);
if isfile(file)
    lines=splitlines(string(fileread(file)));r.solve_result_num=str2double(lines(1));r.message=char(join(lines(2:end),newline));
    r.success=process==0&&r.solve_result_num>=0&&r.solve_result_num<100&&strcmp(lines(2),'solved');
end
end
function writeLog(name,log)
f=fopen(name,'w');fprintf(f,'%s',log);fclose(f);
end
