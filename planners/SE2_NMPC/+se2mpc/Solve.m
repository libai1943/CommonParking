function [q,solver,details]=Solve(c,initial,polygons,o)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));
copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));N=o.intervals;M=polygons.count;
poses=[initial.x,initial.y,unwrap(initial.theta)];mid=(poses(1:end-1,:)+poses(2:end,:))/2;
[lambda,~]=se2mpc.DualSeed(c,mid,polygons);mu=zeros(N,M,2,4);offset=0;
for j=1:M
 A=polygons.A{j};indices=offset+(1:size(A,1));offset=indices(end);normal=lambda(:,indices)*A;
 for side=1:2
  theta=poses(side:side+N-1,3);local=-[cos(theta).*normal(:,1)+sin(theta).*normal(:,2),-sin(theta).*normal(:,1)+cos(theta).*normal(:,2)];
  mu(:,j,side,:)=reshape([max(local(:,1),0),max(-local(:,1),0),max(local(:,2),0),max(-local(:,2),0)],N,1,1,4);
 end
end
counts=cellfun(@(p)size(p,1),polygons.vertices);E=sum(counts);last=cumsum(counts);first=last-counts+1;
v=c.vehicle;A=vertcat(polygons.A{:});b=vertcat(polygons.b{:});g=[v.lw+v.lf,v.lr,v.lb/2,v.lb/2];
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'param radius:=%.17g;\n',hypot(max(v.lw+v.lf,v.lr),v.lb/2));
names={'N','M','E','lw','vmax','amax','phimax','wmax','dmin','min_dt','speed_cost'};
values=[N,M,E,v.lw,v.vmax,v.amax,v.phimax,v.wmax,o.clearance,o.minimumDt,o.speedCost];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
fprintf(f,'param: first last :=\n');fprintf(f,'%d %d %d\n',[(1:M)' first last]');fprintf(f,';\n');
fprintf(f,'param A: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:E)' A]');fprintf(f,';\n');
writeVector(f,'b',b);writeVector(f,'g',g);task=c.task;writeVector(f,'boundary',[task.x0 task.y0 task.theta0 task.xf task.yf task.thetaf]);clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));fprintf(f,'let h:=%.17g;\n',initial.t(end)/N);
for key={'x','y','theta'},name=key{1};fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(0:N)' initial.(name)]');end
for key={'v','phi'},name=key{1};z=initial.(name);fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(0:N)' z]');end
for j=1:E,fprintf(f,['let lambda[%d,',num2str(j),']:=%.17g;\n'],[(0:N-1)' lambda(:,j)]');end
for j=1:M,for side=1:2,for k=1:4,fprintf(f,['let mu[%d,',num2str(j),',',num2str(side),',',num2str(k),']:=%.17g;\n'],[(0:N-1)' mu(:,j,side,k)]');end,end,end
clear closer;
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.9g acceptable_tol=%.9g max_iter=%d max_cpu_time=%.9g print_level=3 linear_solver=ma97 mu_strategy=adaptive";\n',o.solverTolerance,o.solverTolerance,o.maxIterations,o.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g %%.17g\\n",h,cost > "cost.txt";\n');
fprintf(f,'printf {i in 0..N} "%%.17g %%.17g %%.17g\\n",x[i],y[i],theta[i] > "poses.txt";\n');
fprintf(f,'printf {i in 0..N} "%%.17g %%.17g\\n",v[i],phi[i] > "controls.txt";\n');
fprintf(f,'printf {i in 0..N-1,k in 1..E} "%%.17g\\n",lambda[i,k] > "lambda.txt";\n');
fprintf(f,'printf {i in 0..N-1,j in 1..M,s in 1..2,k in 1..4} "%%.17g\\n",mu[i,j,s,k] > "mu.txt";\n');clear closer;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*o.maxCpuSeconds+30);f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);q=[];details=struct();
if isfile(fullfile(folder,'status.txt'))
 lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
 solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
if ~all(cellfun(@(name)isfile(fullfile(folder,name)),{'poses.txt','controls.txt','cost.txt'})),solver.success=false;return;end
p=readmatrix(fullfile(folder,'poses.txt'),'FileType','text');u=readmatrix(fullfile(folder,'controls.txt'),'FileType','text');cost=readmatrix(fullfile(folder,'cost.txt'),'FileType','text');
if ~isequal(size(p),[N+1 3])||~isequal(size(u),[N+1 2])||numel(cost)~=2|| ...
 ~isreal(p)||~isreal(u)||~isreal(cost)||any(~isfinite([p(:);u(:);cost(:)]))||cost(1)<=0
 solver.success=false;solver.message='Invalid native state/control/time output.';return;
end
if M>0
 l=readmatrix(fullfile(folder,'lambda.txt'),'FileType','text');m=readmatrix(fullfile(folder,'mu.txt'),'FileType','text');
 if numel(l)~=N*E||numel(m)~=N*M*8||any(~isfinite([l(:);m(:)]))||~isreal(l)||~isreal(m),solver.success=false;return;end
 lambda=reshape(l,E,N)';mu=permute(reshape(m,4,2,M,N),[4 3 2 1]);
else,lambda=zeros(N,0);mu=zeros(N,0,2,4);end
h=cost(1);check=se2mpc.Check(c,p,u,h,lambda,mu,polygons,o);solver.objective=cost(2);solver.check=check;solver.objective_error=abs(check.objective-cost(2));solver.success=solver.success&&check.success&&solver.objective_error<=1e-6*max(1,abs(cost(2)));
velocity=u(:,1);phi=u(:,2);q=struct('t',(0:N)'*h,'x',p(:,1),'y',p(:,2),'theta',unwrap(p(:,3)), ...
 'v',velocity,'phi',phi,'a',[diff(velocity)/h;0],'omega',[diff(phi)/h;0]);
details=struct('poses',p,'controls',u,'h',h,'lambda',lambda,'mu',mu,'check',check);
end
function writeVector(f,name,z)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(z))' z(:)]');fprintf(f,';\n');
end
