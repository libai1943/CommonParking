function [trajectory,solver,native]=Solve(c,initial,o)
% Triangle21's lifted corner variables, Euler equations and area inequalities.
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));copyfile(fullfile(root,'model.mod'),folder);
v=c.vehicle;N=o.nodes;polygons=parking.PolygonData(c);M=polygons.count;
p=polygons.vertices;counts=cellfun(@(q)size(q,1),p);last=cumsum(counts);first=last-counts+1;vertices=vertcat(p{:});V=size(vertices,1);following=(2:V+1)';following(last)=first;
areas=cellfun(@(q)polyarea(q(:,1),q(:,2)),p);task=c.task;goalTheta=task.thetaf+2*pi*round((initial.theta(end)-task.thetaf)/(2*pi));
parameters=[v.lw;v.lf;v.lr;v.lb;v.v_forward;-v.v_reverse;v.a_accel;-v.a_brake;v.phimax;v.wmax;N;V;task.x0;task.y0;task.theta0;task.xf;task.yf;goalTheta;0];
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
writeVector(f,'BasicParameters',parameters);fprintf(f,'param M:=%d;\nparam area_epsilon:=%.17g;\n',M,o.areaMargin);
writeVector(f,'first',first);writeVector(f,'last',last);writeVector(f,'following',following);writeVector(f,'area',areas);
fprintf(f,'param Obstacle: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:V)' vertices]');fprintf(f,';\n');clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));fprintf(f,'let tf:=%.17g;\n',initial.t(end));
fields={'x','y','theta','v','phi'};
for k=1:5,fprintf(f,['let ',fields{k},'[%d]:=%.17g;\n'],[(1:N)' initial.(fields{k})]');end
h=initial.t(end)/(N-1);a=[diff(initial.v)/h;0];w=[diff(initial.phi)/h;0];
fprintf(f,'let a[%d]:=%.17g;\n',[(1:N)' a]');fprintf(f,'let w[%d]:=%.17g;\n',[(1:N)' w]');
body=[v.lw+v.lf v.lb/2;v.lw+v.lf -v.lb/2;-v.lr -v.lb/2;-v.lr v.lb/2];
for j=1:4
 label=char('A'+j-1);X=initial.x+body(j,1)*cos(initial.theta)-body(j,2)*sin(initial.theta);Y=initial.y+body(j,1)*sin(initial.theta)+body(j,2)*cos(initial.theta);
 fprintf(f,['let ',label,'X[%d]:=%.17g;\n'],[(1:N)' X]');fprintf(f,['let ',label,'Y[%d]:=%.17g;\n'],[(1:N)' Y]');
end
clear closer;
f=fopen(fullfile(folder,'ipopt.opt'),'w');fprintf(f,'max_cpu_time %.9g\nmax_iter %d\ntol 1e-10\nbound_push 0.0001\nmu_strategy adaptive\nlinear_solver ma97\nprint_level 3\n',o.maxCpuSeconds,o.maxIterations);fclose(f);
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\nsolve;\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'printf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g\\n",tf > "time.txt";\nprintf {i in 1..nfe} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",x[i],y[i],theta[i],v[i],phi[i],a[i],w[i] > "states.txt";\n');
fprintf(f,'printf {i in 1..nfe} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",AX[i],AY[i],BX[i],BY[i],CX[i],CY[i],DX[i],DY[i] > "corners.txt";\n');clear closer;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*o.maxCpuSeconds+30);f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
trajectory=[];native=struct();solver=struct('success',false,'native_converged',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);
if isfile(fullfile(folder,'status.txt'))
 lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
 solver.native_converged=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<200&&(contains(lower(solver.message),'optimal')||contains(lower(solver.message),'acceptable'));
end
if isfile(fullfile(folder,'states.txt'))&&isfile(fullfile(folder,'time.txt'))&&isfile(fullfile(folder,'corners.txt'))
 q=readmatrix(fullfile(folder,'states.txt'),'FileType','text');corners=readmatrix(fullfile(folder,'corners.txt'),'FileType','text');tf=readmatrix(fullfile(folder,'time.txt'),'FileType','text');
 if isequal(size(q),[N 7])&&isequal(size(corners),[N 8])&&isscalar(tf)&&isreal(q)&&isreal(corners)&&isreal(tf)&&all(isfinite(q),'all')&&all(isfinite(corners),'all')&&isfinite(tf)&&tf>0
  h=tf/(N-1);rhs=[q(1:end-1,4).*cos(q(1:end-1,3)),q(1:end-1,4).*sin(q(1:end-1,3)),q(1:end-1,4).*tan(q(1:end-1,5))/v.lw,q(1:end-1,6:7)];
  solver.maximum_dynamics_residual=max(abs(diff(q(:,1:5))-h*rhs),[],'all');
  exact=zeros(N,8);for j=1:4,exact(:,2*j-1)=q(:,1)+body(j,1)*cos(q(:,3))-body(j,2)*sin(q(:,3));exact(:,2*j)=q(:,2)+body(j,1)*sin(q(:,3))+body(j,2)*cos(q(:,3));end
  solver.maximum_corner_residual=max(abs(exact-corners),[],'all');
  [safe,gap]=parking.FootprintClearance(q(:,1:3),c,0);solver.node_collision_count=nnz(gap<=0);solver.minimum_node_separation=min(gap);
  solver.success=solver.native_converged&&solver.maximum_dynamics_residual<1e-6&&solver.maximum_corner_residual<1e-6&&safe;
  if solver.native_converged&&~safe,solver.message=[solver.message,' Full-body node collision check failed.'];end
  native=struct('states',q(:,1:5),'controls',q(:,6:7),'corners',corners,'tf',tf);
  trajectory=struct('t',linspace(0,tf,N)','x',q(:,1),'y',q(:,2),'theta',q(:,3),'v',q(:,4),'phi',q(:,5),'a',q(:,6),'omega',q(:,7));
 else,solver.message='Invalid solver output.';end
end
end
function writeVector(f,name,x)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(x))' x(:)]');fprintf(f,';\n');
end
