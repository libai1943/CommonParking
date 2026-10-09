function [q,solver,native]=Solve(c,z,u,h,planes,o)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));
copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));N=o.intervals;v=c.vehicle;polygons=parking.PolygonData(c);M=polygons.count;
counts=cellfun(@(p)size(p,1),polygons.vertices);last=cumsum(counts);first=last-counts+1;P=vertcat(polygons.vertices{:});
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
names={'N','M','E','lw','vmax','amax','phimax','wmax','normal_min','min_time','max_time','time_weight','acceleration_weight','rate_weight'};
values=[N,M,size(P,1),v.lw,v.vmax,v.amax,v.phimax,v.wmax,o.normalMinimum,o.minimumTime,o.maximumTime,o.timeWeight,o.accelerationWeight,o.steeringRateWeight];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
fprintf(f,'param: first last :=\n');fprintf(f,'%d %d %d\n',[(1:M)' first last]');fprintf(f,';\n');
fprintf(f,'param: ox oy :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:size(P,1))' P]');fprintf(f,';\n');
writeVector(f,'bx',[v.lw+v.lf,v.lw+v.lf,-v.lr,-v.lr]);writeVector(f,'body_y',v.lb/2*[1 -1 -1 1]);
t=c.task;writeVector(f,'boundary',[t.x0 t.y0 t.theta0 t.xf t.yf t.thetaf]);clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));fprintf(f,'let h:=%.17g;\n',h);
names={'x','y','theta','v','phi'};
for j=1:5,fprintf(f,['let ',names{j},'[%d]:=%.17g;\n'],[(0:N)' z(:,j)]');end
names={'a','omega'};
for j=1:2,fprintf(f,['let ',names{j},'[%d]:=%.17g;\n'],[(0:N-1)' u(:,j)]');end
names={'nx','ny','mu'};
for j=1:M,for k=1:3,fprintf(f,['let ',names{k},'[%d,',num2str(j),']:=%.17g;\n'],[(0:N)' planes(:,j,k)]');end,end
clear closer;
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.9g acceptable_tol=%.9g max_iter=%d max_cpu_time=%.9g print_level=3 linear_solver=ma97 mu_strategy=adaptive";\n',o.solverTolerance,o.solverTolerance,o.maxIterations,o.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g %%.17g\\n",h,cost > "cost.txt";\n');
fprintf(f,'printf {i in 0..N} "%%.17g %%.17g %%.17g %%.17g %%.17g\\n",x[i],y[i],theta[i],v[i],phi[i] > "states.txt";\n');
fprintf(f,'printf {i in 0..N-1} "%%.17g %%.17g\\n",a[i],omega[i] > "controls.txt";\n');
fprintf(f,'printf {i in 0..N,j in 1..M} "%%.17g %%.17g %%.17g\\n",nx[i,j],ny[i,j],mu[i,j] > "planes.txt";\n');clear closer;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*o.maxCpuSeconds+30);f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);q=[];native=struct();
if isfile(fullfile(folder,'status.txt'))
 lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
 solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
if ~all(cellfun(@(name)isfile(fullfile(folder,name)),{'states.txt','controls.txt','cost.txt'})),solver.success=false;return;end
z=readmatrix(fullfile(folder,'states.txt'),'FileType','text');u=readmatrix(fullfile(folder,'controls.txt'),'FileType','text');cost=readmatrix(fullfile(folder,'cost.txt'),'FileType','text');
if M>0
 p=readmatrix(fullfile(folder,'planes.txt'),'FileType','text');
 if ~isequal(size(p),[(N+1)*M 3])||~isreal(p)||any(~isfinite(p),'all'),solver.success=false;solver.message='Invalid separating hyperplanes.';return;end
 planes=permute(reshape(p,M,N+1,3),[2 1 3]);
else,planes=zeros(N+1,0,3);end
if ~isequal(size(z),[N+1 5])||~isequal(size(u),[N 2])||numel(cost)~=2|| ...
 ~isreal(z)||~isreal(u)||~isreal(cost)||any(~isfinite([z(:);u(:);cost(:)]))||cost(1)<=0
 solver.success=false;solver.message='Invalid native state/control/time output.';return;
end
h=cost(1);check=hpocp.Check(c,z,u,h,planes,o);solver.objective=cost(2);solver.check=check;solver.success=solver.success&&check.success;
q=struct('t',(0:N)'*h,'x',z(:,1),'y',z(:,2),'theta',unwrap(z(:,3)),'v',z(:,4),'phi',z(:,5),'a',[u(:,1);u(end,1)],'omega',[u(:,2);u(end,2)]);
native=struct('states',z,'controls',u,'h',h,'planes',planes,'check',check);
end
function writeVector(f,name,z)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(z))' z(:)]');fprintf(f,';\n');
end
