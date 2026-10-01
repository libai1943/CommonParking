function [trajectory,solver,native] = Solve(c,initial,nodes,D,options)
% Fully implicit finite-element collocation of the literal 2015 equations.
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();
root=fileparts(fileparts(mfilename('fullpath')));copyfile(fullfile(root,'model.mod'),folder);
v=c.vehicle;E=options.elements;polygons=parking.PolygonData(c);M=polygons.count;p=polygons.vertices;
counts=cellfun(@(q)size(q,1),p);last=cumsum(counts);first=last-counts+1;vertices=vertcat(p{:});V=size(vertices,1);
following=(2:V+1)';following(last)=first;
areas=cellfun(@(q)polyarea(q(:,1),q(:,2)),p);
body=[v.lf v.lb/2;v.lf -v.lb/2;-(v.lw+v.lr) -v.lb/2;-(v.lw+v.lr) v.lb/2];
t=c.task;boundary=[t.x0+v.lw*cos(t.theta0) t.y0+v.lw*sin(t.theta0) t.theta0 ...
    t.xf+v.lw*cos(t.thetaf) t.yf+v.lw*sin(t.thetaf) initial.theta(end)];
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
names={'NE','M','V','lw','total_length','width','vmax','amax','phimax','wmax','area_margin','max_time'};
values=[E M V v.lw v.lw+v.lf+v.lr v.lb v.vmax v.amax v.phimax v.wmax options.areaMargin options.maxTime];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
fprintf(f,'param D: 0 1 2 3 :=\n');fprintf(f,'%d %.17g %.17g %.17g %.17g\n',[(1:3)' D]');fprintf(f,';\n');
writeVector(f,'tau',nodes,0);writeVector(f,'first',first,1);writeVector(f,'last',last,1);
writeVector(f,'following',following,1);writeVector(f,'area',areas,1);writeVector(f,'boundary',boundary,1);
fprintf(f,'param obstacle: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:V)' vertices]');fprintf(f,';\n');
fprintf(f,'param body: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:4)' body]');fprintf(f,';\n');clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'let tf:=%.17g;\n',min(initial.t(end),options.maxTime));
fields={'x','y','theta','v','phi'};h=initial.t(end)/E;
for e=1:E
    indices=(e-1)*4+(1:4);
    for s=1:5
        fprintf(f,['let z[',num2str(e),',%d,',num2str(s),']:=%.17g;\n'],[(0:3)' initial.(fields{s})(indices)]');
    end
    a=min(max(initial.a(indices(2:4)),-v.amax),v.amax);
    omega=min(max(D*initial.phi(indices)/h,-v.wmax),v.wmax);
    fprintf(f,['let u[',num2str(e),',%d,1]:=%.17g;\n'],[(1:3)' a]');
    fprintf(f,['let u[',num2str(e),',%d,2]:=%.17g;\n'],[(1:3)' omega]');
end
clear closer;
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=1e-12 acceptable_tol=1e-12 bound_push=1e-4 max_iter=%d max_cpu_time=%.9g print_level=3 mu_strategy=adaptive linear_solver=ma97";\n',options.maxIterations,options.maxCpuSeconds);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g\\n",tf > "time.txt";\n');
fprintf(f,'printf {e in 1..NE,k in 0..3} "%%.17g %%.17g %%.17g %%.17g %%.17g\\n",z[e,k,1],z[e,k,2],z[e,k,3],z[e,k,4],z[e,k,5] > "states.txt";\n');
fprintf(f,'printf {e in 1..NE,k in 1..3} "%%.17g %%.17g\\n",u[e,k,1],u[e,k,2] > "controls.txt";\n');clear closer;
old=pwd;restore=onCleanup(@()cd(old));cd(folder);
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*options.maxCpuSeconds+30);
f=fopen('solver.log','w');fprintf(f,'%s',log);fclose(f);
trajectory=[];native=struct();solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);
if isfile('status.txt')
    lines=splitlines(string(fileread('status.txt')));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
    solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
if isfile('states.txt')&&isfile('controls.txt')&&isfile('time.txt')
    z=readmatrix('states.txt','FileType','text');u=readmatrix('controls.txt','FileType','text');tf=readmatrix('time.txt','FileType','text');
    if isequal(size(z),[4*E 5])&&isequal(size(u),[3*E 2])&&isscalar(tf)&&isreal(z)&&isreal(u)&&isreal(tf) ...
            &&all(isfinite(z),'all')&&all(isfinite(u),'all')&&isfinite(tf)&&tf>0
        native=struct('states',z,'controls',u,'tf',tf,'nodes',nodes,'D',D,'elements',E);
        % Keep each element's three stages and one global initial state.
        indices=[1;reshape((0:E-1)*4+(2:4)',[],1)];q=z(indices,:);
        time=[0;reshape(((0:E-1)+nodes(2:4))/E,[],1)]*tf;
        % Extrapolate the first quadratic control polynomial to t=0.
        basis=zeros(1,3);
        for j=1:3
            others=nodes(setdiff(2:4,j+1));basis(j)=prod(-others)/prod(nodes(j+1)-others);
        end
        controls=[basis*u(1:3,:);u];
        trajectory=struct('t',time,'x',q(:,1)-v.lw*cos(q(:,3)),'y',q(:,2)-v.lw*sin(q(:,3)), ...
            'theta',q(:,3),'v',q(:,4),'phi',q(:,5),'a',controls(:,1),'omega',controls(:,2));
    else,solver.success=false;solver.message='Invalid solver output.';end
else,solver.success=false;solver.message=[solver.message,' Complete solver output missing.'];end
end
function writeVector(f,name,x,start)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(start:start+numel(x)-1)' x(:)]');fprintf(f,';\n');
end
