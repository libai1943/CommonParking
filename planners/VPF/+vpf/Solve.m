function [q,solver,native]=Solve(c,initial,alpha,o)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));
copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));N=o.intervals;v=c.vehicle;polygons=parking.PolygonData(c);
C=zeros(0,2);D=C;
for j=1:polygons.count,P=polygons.vertices{j};C=[C;P];D=[D;P([2:end 1],:)];end %#ok<AGROW>
segments=zeros(4*(2*N+1),4);row=0;
for i=0:N,for j=1:4,row=row+1;segments(row,:)=[i i j mod(j,4)+1];end,end
for i=0:N-1,for j=1:4,row=row+1;segments(row,:)=[i i+1 j j];end,end
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
names={'N','E','S','lw','vmax','amax','phimax','wmax','jerkmax','alpha','min_time','max_time','segment_threshold'};
values=[N,size(C,1),row,v.lw,v.vmax,v.amax,v.phimax,v.wmax,o.jerkLimit,alpha,o.minimumTime,o.maximumTime,o.segmentThreshold];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
fprintf(f,'param: n1 n2 j1 j2 :=\n');fprintf(f,'%d %d %d %d %d\n',[(1:row)' segments]');fprintf(f,';\n');
fprintf(f,'param: cx cy dx dy :=\n');fprintf(f,'%d %.17g %.17g %.17g %.17g\n',[(1:size(C,1))' C D]');fprintf(f,';\n');
writeVector(f,'bx',[v.lw+v.lf,v.lw+v.lf,-v.lr,-v.lr]);writeVector(f,'body_y',v.lb/2*[1 -1 -1 1]);
t=c.task;writeVector(f,'boundary',[t.x0 t.y0 t.theta0 t.xf t.yf t.thetaf]);clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));fprintf(f,'let h:=%.17g;\n',initial.t(end)/N);
for key={'x','y','theta','v'},name=key{1};fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(0:N)' initial.(name)]');end
for key={'a','phi'},name=key{1};z=initial.(name);fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(0:N-1)' z(1:N)]');end
clear closer;
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.9g acceptable_tol=%.9g max_iter=%d max_cpu_time=%.9g print_level=3 linear_solver=ma97 mu_strategy=adaptive";\n',o.solverTolerance,o.solverTolerance,o.maxIterations,o.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g %%.17g\\n",h,cost > "cost.txt";\n');
fprintf(f,'printf {i in 0..N} "%%.17g %%.17g %%.17g %%.17g\\n",x[i],y[i],theta[i],v[i] > "states.txt";\n');
fprintf(f,'printf {i in 0..N-1} "%%.17g %%.17g\\n",a[i],phi[i] > "controls.txt";\n');clear closer;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*o.maxCpuSeconds+30);f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder,'alpha',alpha);q=[];native=struct();
if isfile(fullfile(folder,'status.txt'))
 lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
 solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
if ~all(cellfun(@(name)isfile(fullfile(folder,name)),{'states.txt','controls.txt','cost.txt'})),solver.success=false;return;end
z=readmatrix(fullfile(folder,'states.txt'),'FileType','text');u=readmatrix(fullfile(folder,'controls.txt'),'FileType','text');cost=readmatrix(fullfile(folder,'cost.txt'),'FileType','text');
if ~isequal(size(z),[N+1 4])||~isequal(size(u),[N 2])||numel(cost)~=2|| ...
 ~isreal(z)||~isreal(u)||~isreal(cost)||any(~isfinite([z(:);u(:);cost(:)]))||cost(1)<=0
 solver.success=false;solver.message='Invalid native state/control/time output.';return;
end
h=cost(1);check=vpf.Check(c,z,u,h,alpha,o);solver.objective=cost(2);solver.check=check;solver.success=solver.success&&check.success;
q=struct('t',(0:N)'*h,'x',z(:,1),'y',z(:,2),'theta',unwrap(z(:,3)),'v',z(:,4),'a',[u(:,1);u(end,1)], ...
 'phi',[u(:,2);u(end,2)],'omega',[diff(u(:,2))/h;0;0]);
native=struct('states',z,'controls',u,'h',h,'alpha',alpha,'check',check);
end
function writeVector(f,name,z)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(z))' z(:)]');fprintf(f,';\n');
end
