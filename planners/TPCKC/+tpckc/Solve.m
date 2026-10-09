function [q,solver,native]=Solve(c,reference,keys,seeds,radius,o,goalHeading)
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));copyfile(fullfile(root,'model.mod'),fullfile(folder,'model.mod'));
v=c.vehicle;N=numel(reference.x)-1;M=size(keys,1);polys=parking.PolygonData(c);body=[v.lw+v.lf,v.lb/2;v.lw+v.lf,-v.lb/2;-v.lr,-v.lb/2;-v.lr,v.lb/2];G=[1 0;0 1;-1 0;0 -1];g=[v.lw+v.lf;v.lb/2;v.lr;v.lb/2];
f=fopen(fullfile(folder,'data.dat'),'w');close=onCleanup(@()fclose(f));
names={'N','M','lw','vmax','amax','phimax','wmax','radius','minimumTime','maximumTime','minimumDistance','minHeading','maxHeading'};
values=[N,M,v.lw,v.vmax,v.amax,v.phimax,v.wmax,radius,o.minimumTime,o.maximumTime,o.dualDistance,min(c.task.theta0,goalHeading)-pi,max(c.task.theta0,goalHeading)+pi];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
table(f,'weights',o.weights(:),1);table(f,'boundary',[c.task.x0;c.task.y0;c.task.theta0;c.task.xf;c.task.yf;goalHeading],1);table(f,'rx ry',[reference.x reference.y],0);
keyData=zeros(M,5);planes=cell(M,1);
for m=1:M
 i=keys(m,3);j=keys(m,4);if keys(m,2)==1,A=polys.A{i};b=polys.b{i};point=body(j,:);else,A=G;b=g;point=polys.vertices{i}(j,:);end
 keyData(m,:)=[size(A,1),keys(m,1)-1,keys(m,2),point];planes{m}=[A,b];
end
table(f,'J timeIndex kind px py',keyData,1);
for col=1:3
 name={'ax','ay','bound'};fprintf(f,'param %s:=\n',name{col});for m=1:M,for j=1:size(planes{m},1),fprintf(f,'%d %d %.17g\n',m,j,planes{m}(j,col));end;end;fprintf(f,';\n');
end
clear close;
f=fopen(fullfile(folder,'initial.run'),'w');close=onCleanup(@()fclose(f));
tableNames={'zx','zy','ztheta','zv','zphi','za','zw'};z=[reference.x/radius,reference.y/radius,reference.theta/pi,reference.v/v.vmax,reference.phi/v.phimax,reference.a/v.amax,reference.omega/v.wmax];
fprintf(f,'let T:=%.17g;\n',reference.t(end));for j=1:7,count=N+1-(j>=6);fprintf(f,['let ',tableNames{j},'[%d]:=%.17g;\n'],[(0:count-1)' z(1:count,j)]');end
for m=1:M,for j=1:numel(seeds{m}),fprintf(f,'let lambda[%d,%d]:=%.17g;\n',m,j,seeds{m}(j));end;end;clear close;
f=fopen(fullfile(folder,'solve.run'),'w');close=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.9g acceptable_tol=%.9g max_iter=%d max_cpu_time=%.9g print_level=3 linear_solver=ma97 mu_strategy=adaptive";\n',o.solverTolerance,o.solverTolerance,o.maxIterations,o.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g %%.17g\\n",T,cost > "objective.txt";\n');
fprintf(f,'printf {k in I} "%%.17g %%.17g %%.17g %%.17g %%.17g\\n",x[k],y[k],theta[k],v[k],phi[k] > "states.txt";\n');
fprintf(f,'printf {k in U} "%%.17g %%.17g\\n",a[k],omega[k] > "controls.txt";\n');
if M>0,fprintf(f,'printf {m in K,j in 1..J[m]} "%%d %%d %%.17g\\n",m,j,lambda[m,j] > "duals.txt";\n');end;clear close;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*o.maxCpuSeconds+30);f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);q=[];native=struct();
if isfile(fullfile(folder,'status.txt'))
 lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
if ~all(cellfun(@(name)isfile(fullfile(folder,name)),{'states.txt','controls.txt','objective.txt'})),solver.success=false;return;end
x=readmatrix(fullfile(folder,'states.txt'),'FileType','text');u=readmatrix(fullfile(folder,'controls.txt'),'FileType','text');cost=readmatrix(fullfile(folder,'objective.txt'),'FileType','text');
if ~isequal(size(x),[N+1,5])||~isequal(size(u),[N,2])||numel(cost)~=2||~isreal([x(:);u(:);cost(:)])||any(~isfinite([x(:);u(:);cost(:)]))||cost(1)<=0,solver.success=false;solver.message='Invalid native states, controls or time.';return;end
lambda=cell(M,1);if M>0
 if ~isfile(fullfile(folder,'duals.txt')),solver.success=false;return;end
 z=readmatrix(fullfile(folder,'duals.txt'),'FileType','text');if size(z,2)~=3||size(z,1)~=sum(keyData(:,1))||~isreal(z)||any(~isfinite(z),'all'),solver.success=false;return;end
 for m=1:M,ids=find(z(:,1)==m);assert(isequal(z(ids,2),(1:keyData(m,1))'));lambda{m}=z(ids,3);end
end
q=struct('t',linspace(0,cost(1),N+1)','x',x(:,1),'y',x(:,2),'theta',x(:,3),'v',x(:,4),'phi',x(:,5),'a',[u(:,1);u(end,1)],'omega',[u(:,2);u(end,2)]);
check=tpckc.Check(q,lambda,keys,c,reference,radius,o,goalHeading);solver.check=check;solver.objective=cost(2);solver.success=solver.success&&check.success&&abs(check.objective-cost(2))<1e-5;
native=struct('trajectory',q,'keys',keys,'duals',{lambda},'warm_duals',{seeds},'reference',reference,'trust_radius',radius,'goal_heading',goalHeading,'check',check);
end
function table(f,names,z,first)
fprintf(f,'param: %s :=\n',names);format=['%d',repmat(' %.17g',1,size(z,2)),'\n'];fprintf(f,format,[(first:first+size(z,1)-1)' z]');fprintf(f,';\n');
end
