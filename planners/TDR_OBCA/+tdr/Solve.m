function [q,solver,native] = Solve(c,initial,lambda,mu,d,o)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));
copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));
p=parking.PolygonData(c);N=numel(initial.t);M=p.count;counts=cellfun(@(z)size(z,1),p.A);
E=sum(counts);last=cumsum(counts);first=last-counts+1;A=vertcat(p.A{:});b=vertcat(p.b{:});v=c.vehicle;
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
names={'N','M','E','lw','vmax','amax','phimax','wmax','epsilon','h'};
values=[N,M,E,v.lw,v.vmax,v.amax,v.phimax,v.wmax,o.strictDistance,initial.t(end)/(N-1)];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
fprintf(f,'param: first last :=\n');fprintf(f,'%d %d %d\n',[(1:M)',first,last]');fprintf(f,';\n');
fprintf(f,'param A: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:E)',A]');fprintf(f,';\n');
vector(f,'b',b);vector(f,'g',[v.lw+v.lf,v.lr,v.lb/2,v.lb/2]);vector(f,'weights',o.weights);
vector(f,'boundary',[c.task.x0,c.task.y0,c.task.theta0,0,c.task.xf,c.task.yf,initial.theta(end),0]);clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));
for key={'x','y','theta','v','phi','a'}
 name=key{1};n=N-ismember(name,{'phi','a'});fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(1:n)',initial.(name)(1:n)]');
end
for e=1:E,fprintf(f,['let lambda[%d,',num2str(e),']:=%.17g;\n'],[(2:N)',lambda(2:N,e)]');end
for j=1:M
 fprintf(f,['let d[%d,',num2str(j),']:=%.17g;\n'],[(2:N)',d(2:N,j)]');
 for k=1:4,fprintf(f,['let mu[%d,',num2str(j),',',num2str(k),']:=%.17g;\n'],[(2:N)',mu(2:N,j,k)]');end
end
clear closer;
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.9g acceptable_tol=%.9g max_iter=%d max_cpu_time=%.9g print_level=3 linear_solver=ma97 mu_strategy=adaptive";\n',o.tolerance,o.tolerance,o.maxIterations,o.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g\\n",cost > "objective.txt";\n');
fprintf(f,'printf {i in 1..N} "%%.17g %%.17g %%.17g %%.17g\\n",x[i],y[i],theta[i],v[i] > "states.txt";\n');
fprintf(f,'printf {i in 1..N-1} "%%.17g %%.17g\\n",phi[i],a[i] > "controls.txt";\n');
fprintf(f,'printf {i in 2..N,j in 1..E} "%%.17g\\n",lambda[i,j] > "lambda.txt";\n');
fprintf(f,'printf {i in 2..N,j in 1..M,k in 1..4} "%%.17g\\n",mu[i,j,k] > "mu.txt";\n');
fprintf(f,'printf {i in 2..N,j in 1..M} "%%.17g\\n",d[i,j] > "d.txt";\n');clear closer;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*o.maxCpuSeconds+30);
f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);q=[];native=struct();
if isfile(fullfile(folder,'status.txt'))
 lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
 solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
files={'states','controls','lambda','mu','d','objective'};data=cell(size(files));
for j=1:numel(files)
 file=fullfile(folder,[files{j},'.txt']);if ~isfile(file),solver.success=false;return;end
 data{j}=readmatrix(file,'FileType','text');if ~isreal(data{j})||any(~isfinite(data{j}),'all'),solver.success=false;solver.message='Invalid native output.';return;end
end
X=data{1};U=data{2};assert(isequal(size(X),[N,4])&&isequal(size(U),[N-1,2]));
lambda=reshape(data{3},E,N-1)';mu=permute(reshape(data{4},4,M,N-1),[3,2,1]);d=reshape(data{5},M,N-1)';
phi=[U(:,1);U(end,1)];q=struct('t',initial.t,'x',X(:,1),'y',X(:,2),'theta',X(:,3),'v',X(:,4), ...
 'phi',phi,'a',[U(:,2);0],'omega',[diff(phi)./diff(initial.t);0]);
check=tdr.Check(c,q,lambda,mu,d,o,initial.theta(end));solver.check=check;solver.objective=data{6};
solver.success=solver.success&&check.success&&abs(check.objective-data{6})<1e-5*max(1,abs(data{6}));
native=struct('lambda',lambda,'mu',mu,'d',d,'check',check,'goal_heading',initial.theta(end));
end
function vector(f,name,z)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(z))',z(:)]');fprintf(f,';\n');
end
