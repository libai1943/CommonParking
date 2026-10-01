function [trajectory,solver,native] = Solve(c,options)
% Algorithm 1: constant initial guess, decreasing MAKKT relaxation, PSOC NLP.
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();root=fileparts(fileparts(mfilename('fullpath')));
copyfile(fullfile(root,'model.mod'),folder);N=options.nodes;v=c.vehicle;polygons=parking.PolygonData(c);
[tau,D,weight,barycentric]=cp.LegendreLobatto(N);M=polygons.count;counts=cellfun(@(p)size(p,1),polygons.vertices);
body=[v.lw+v.lf v.lb/2;v.lw+v.lf -v.lb/2;-v.lr -v.lb/2;-v.lr v.lb/2];
t=c.task;goalTheta=t.theta0+atan2(sin(t.thetaf-t.theta0),cos(t.thetaf-t.theta0));
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
names={'N','M','lw','vmax','phimax','wmax','delta','max_time'};
values=[N M v.lw v.vmax v.phimax v.wmax options.safetyPseudodistance options.maxTime];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
writeVector(f,'tau',tau);writeVector(f,'weight',weight);writeVector(f,'NV',counts);
writeVector(f,'boundary',[t.x0 t.y0 t.theta0 t.xf t.yf goalTheta]);
fprintf(f,'param D: ');fprintf(f,'%d ',1:N);fprintf(f,':=\n');
fprintf(f,['%d',repmat(' %.17g',1,N),'\n'],[(1:N)' D]');fprintf(f,';\n');
fprintf(f,'param body: 1 2 :=\n');fprintf(f,'%d %.17g %.17g\n',[(1:4)' body]');fprintf(f,';\nparam obstacle:=\n');
for m=1:M,for j=1:counts(m),for k=1:2,fprintf(f,'%d %d %d %.17g\n',m,j,k,polygons.vertices{m}(j,k));end,end,end
fprintf(f,';\n');clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'let tf:=%.17g;\n',options.initialTime);
fprintf(f,'let {i in 1..N} x[i]:=%.17g;\nlet {i in 1..N} y[i]:=%.17g;\nlet {i in 1..N} theta[i]:=%.17g;\n',t.x0,t.y0,t.theta0);
% All other variables default to zero, as in the paper's Section 4.1.
clear closer;old=pwd;restore=onCleanup(@()cd(old));cd(folder);
trajectory=[];native=struct();history=cell(0,1);previous=NaN;converged=false;
solver=struct('success',false,'solve_result_num',NaN,'message','','working_directory',folder);
for iteration=1:numel(options.epsilon)
    relaxation=options.epsilon(iteration);
    f=fopen('solve.run','w');closer=onCleanup(@()fclose(f));
    fprintf(f,'reset;model model.mod;data data.dat;let epsilon:=%.17g;\n',relaxation);
    if iteration==1,fprintf(f,'include initial.run;\n');else,fprintf(f,'include restart.run;\n');end
    fprintf(f,'option solver "%s";\n',strrep(runtime.ipopt,'\','/'));
    fprintf(f,['option ipopt_options "tol=1e-12 acceptable_tol=1e-16 constr_viol_tol=1e-12 acceptable_constr_viol_tol=1e-12 ', ...
        'compl_inf_tol=1e-4 acceptable_compl_inf_tol=1e-4 dual_inf_tol=1 acceptable_dual_inf_tol=1 ', ...
        'max_iter=%d max_cpu_time=%.9g print_level=3 mu_strategy=adaptive linear_solver=ma86";\n'],options.maxIterations,options.maxCpuSeconds);
    fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
    fprintf(f,'printf "%%.17g %%.17g\\n",tf,cost > "cost.txt";\n');
    fprintf(f,'printf {i in 1..N} "%%.17g %%.17g %%.17g %%.17g %%.17g %%.17g\\n",x[i],y[i],theta[i],phi[i],v[i],omega[i] > "candidate.txt";\n');
    fprintf(f,'printf {i in 1..N,m in 1..M,j in 1..NV[m]+8} "%%.17g %%.17g\\n",p[i,m,j],lambda[i,m,j] > "primal_dual.txt";\n');
    fprintf(f,'printf {i in 1..N,m in 1..M,k in 1..4} "%%.17g\\n",nu[i,m,k] > "equality_dual.txt";\n');
    fprintf(f,'printf "let tf:=%%.17g;\\n",tf > "restart.run";\n');
    for key={'x','y','theta','phi','v','omega'}
        name=key{1};fprintf(f,['printf {i in 1..N} "let ',name,'[%%d]:=%%.17g;\\n",i,',name,'[i] >> "restart.run";\n']);
    end
    for key={'p','lambda'}
        name=key{1};fprintf(f,['printf {i in 1..N,m in 1..M,j in 1..NV[m]+8} "let ',name,'[%%d,%%d,%%d]:=%%.17g;\\n",i,m,j,',name,'[i,m,j] >> "restart.run";\n']);
    end
    fprintf(f,'printf {i in 1..N,m in 1..M,k in 1..4} "let nu[%%d,%%d,%%d]:=%%.17g;\\n",i,m,k,nu[i,m,k] >> "restart.run";\n');clear closer;
    [process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*options.maxCpuSeconds+30);
    f=fopen(sprintf('solver_%d.log',iteration),'w');fprintf(f,'%s',log);fclose(f);
    entry=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'epsilon',relaxation,'message',log);
    if isfile('status.txt')
        lines=splitlines(string(fileread('status.txt')));entry.solve_result_num=str2double(lines(1));entry.message=char(join(lines(2:end),newline));
        entry.success=process==0&&entry.solve_result_num>=0&&entry.solve_result_num<100&&strcmp(lines(2),'solved');
    end
    if isfile('candidate.txt')&&isfile('cost.txt')&&isfile('primal_dual.txt')&&isfile('equality_dual.txt')
        z=readmatrix('candidate.txt','FileType','text');cost=readmatrix('cost.txt','FileType','text');
        pd=readmatrix('primal_dual.txt','FileType','text');nu=readmatrix('equality_dual.txt','FileType','text');
        if isequal(size(z),[N 6])&&numel(cost)==2&&isreal(z)&&isreal(cost)&&all(isfinite(z),'all')&&all(isfinite(cost)) ...
                &&cost(1)>0&&isequal(size(pd),[N*sum(counts+8) 2])&&isequal(size(nu),[N*M*4 1]) ...
                &&isreal(pd)&&isreal(nu)&&all(isfinite(pd),'all')&&all(isfinite(nu))
            tf=cost(1);entry.objective=cost(2);entry.objective_relative_change=abs(cost(2)-previous)/(1+abs(previous));
            native=struct('states_controls',z,'primal_dual',pd,'equality_dual',nu,'tf',tf, ...
                'nodes',tau,'D',D,'quadrature_weights',weight,'epsilon',relaxation);
            % Expose the global polynomials, not linear chords through 15 nodes.
            time=unique([linspace(0,tf,max(2,ceil(tf/options.outputStep)+1))';(tau+1)*tf/2]);
            q=cp.BarycentricInterpolate(tau,barycentric,z,2*time/tf-1);
            acceleration=cp.BarycentricInterpolate(tau,barycentric,2/tf*D*z(:,5),2*time/tf-1);
            trajectory=struct('t',time,'x',q(:,1),'y',q(:,2),'theta',q(:,3),'v',q(:,5),'phi',q(:,4),'a',acceleration,'omega',q(:,6));
            converged=entry.success&&iteration>1&&entry.objective_relative_change<=options.objectiveTolerance;
            previous=cost(2);
        else,entry.success=false;entry.message='Invalid solver output.';end
    else,entry.success=false;entry.message=[entry.message,' Complete solver output missing.'];end
    history{end+1}=entry; %#ok<AGROW>
    solver.solve_result_num=entry.solve_result_num;solver.message=entry.message;
    if ~entry.success||converged,break;end
end
solver.success=converged;solver.iterations=numel(history);solver.attempts=history;
solver.converged_objective=converged;solver.final_epsilon=options.epsilon(numel(history));
if converged,solver.message='Native pseudospectral NLP solved; Algorithm 1 objective-change criterion satisfied.';
elseif entry.success,solver.message='Native NLP solved, but objective-change criterion was not reached within the relaxation sequence.';end
end
function writeVector(f,name,x)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(x))' x(:)]');fprintf(f,';\n');
end
