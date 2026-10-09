function [q,solver,native]=Solve(c,reference,data,o)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));
N=o.nodes;v=c.vehicle;E=data.ego_planes;O=data.obstacle_planes;
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
names={'N','NE','NO','lw','vmax','amax','phimax','wmax','hmin','hmax','wa','ww'};
values=[N,size(E,1),size(O,1),v.lw,v.vmax,v.amax,v.phimax,v.wmax,o.minimumStep,o.maximumStep,o.accelerationWeight,o.steeringRateWeight];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
writeTable(f,'ei ex ey ea eb ec',E);writeTable(f,'oi ox oy oa ob oc',O);
writeTable(f,'rx ry rt dr da',[reference.x reference.y reference.theta data.position_radius data.angle_radius]);
t=c.task;writeTable(f,'boundary',[t.x0;t.y0;t.theta0;t.xf;t.yf;t.thetaf]);clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));
fields={'x','y','theta','v','phi','a','omega'};
for j=1:numel(fields),name=fields{j};fprintf(f,['let ',name,'[%d]:=%.17g;\n'],[(1:N)' reference.(name)]');end
h=max(o.minimumStep,min(o.maximumStep,diff(reference.t)));fprintf(f,'let h[%d]:=%.17g;\n',[(1:N-1)' h]');clear closer;
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.9g acceptable_tol=%.9g max_iter=%d max_cpu_time=%.9g print_level=3 linear_solver=ma97 mu_strategy=adaptive";\n',o.solverTolerance,o.solverTolerance,o.maxIterations,o.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g\\n",cost > "cost.txt";\n');
fprintf(f,'printf {i in 1..N} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",x[i],y[i],theta[i],v[i],phi[i],a[i],omega[i] > "states.txt";\n');
fprintf(f,'printf {i in 1..N-1} "%%.17g\\n",h[i] > "steps.txt";\n');clear closer;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*o.maxCpuSeconds+30);f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);q=[];native=struct();
if isfile(fullfile(folder,'status.txt'))
 lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
 solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
if ~all(cellfun(@(name)isfile(fullfile(folder,name)),{'states.txt','steps.txt','cost.txt'})),solver.success=false;return;end
z=readmatrix(fullfile(folder,'states.txt'),'FileType','text');h=readmatrix(fullfile(folder,'steps.txt'),'FileType','text');cost=readmatrix(fullfile(folder,'cost.txt'),'FileType','text');
if ~isequal(size(z),[N 7])||~isequal(size(h),[N-1 1])||~isscalar(cost)||~isreal(z)||~isreal(h)||~isreal(cost)||any(~isfinite([z(:);h;cost]))||any(h<=0)
 solver.success=false;solver.message='Invalid native state/control/time output.';return;
end
q=struct('t',[0;cumsum(h)],'x',z(:,1),'y',z(:,2),'theta',z(:,3),'v',z(:,4),'phi',z(:,5),'a',z(:,6),'omega',z(:,7));
check=psro.Check(c,q,h,data,reference,o);solver.check=check;solver.objective=cost;solver.success=solver.success&&check.success;
native=struct('trajectory',q,'steps',h,'constraints',data,'reference',reference,'check',check);
end
function writeTable(f,names,z)
fprintf(f,'param: %s :=\n',names);format=['%d',repmat(' %.17g',1,size(z,2)),'\n'];fprintf(f,format,[(1:size(z,1))' z]');fprintf(f,';\n');
end
