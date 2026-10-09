function report = TestLIOMFormulation()
% Algebraic objective equivalence and Hessian sparsity at identical inputs.
root=SetupCommonParking();runtime=cp.AmplRuntime();records=cell(2,1);
for variant=1:2
    folder=cp.RuntimeDirectory();model=fileread(fullfile(root,'planners','LIOM','model.mod'));
    if variant==1
        model=regexprep(model,'(?s)minimize compound_cost:.*?;', ...
            'minimize compound_cost: nominal_cost/weight_penalty+infeasibility;');
    end
    f=fopen(fullfile(folder,'model.mod'),'w');fprintf(f,'%s',model);fclose(f);
    f=fopen(fullfile(folder,'check.run'),'w');
    fprintf(f,'reset;model model.mod;\nlet N:=51;let D:=3;let lw:=2.8;let vmax:=2.5;let amax:=1;let phimax:=0.7;let wmax:=0.5;let max_time:=100;let weight_energy:=0.01;let weight_penalty:=1e9;\n');
    fprintf(f,'let {k in 1..6} boundary[k]:=0;let {j in 1..D} offset[j,1]:=0.2*j;let {j in 1..D} offset[j,2]:=0.1*cos(j);\n');
    fprintf(f,'let {i in 1..N,j in 1..D} box[i,j,1]:=-100;let {i in 1..N,j in 1..D} box[i,j,2]:=100;let {i in 1..N,j in 1..D} box[i,j,3]:=-100;let {i in 1..N,j in 1..D} box[i,j,4]:=100;\n');
    fprintf(f,'for {k in 1..8} {let tf:=7+k;let {i in 1..N} x[i]:=0.2*sin(i+k);let {i in 1..N} y[i]:=0.3*cos(i-k);let {i in 1..N} theta[i]:=0.4*sin(i*k);\n');
    fprintf(f,'let {i in 1..N} v[i]:=0.2*cos(i+k);let {i in 1..N} phi[i]:=0.3*sin(i-k);let {i in 1..N} a[i]:=0.1*sin(i*k);let {i in 1..N} w[i]:=0.1*cos(i*k);\n');
    fprintf(f,'let {i in 1..N,j in 1..D} cx[i,j]:=x[i]+offset[j,1]+0.03*sin(i*j);let {i in 1..N,j in 1..D} cy[i,j]:=y[i]+offset[j,2]+0.03*cos(i*j);\n');
    % Keep presolve-fixed endpoints consistent when reassigning trial values.
    fprintf(f,'let x[1]:=0;let x[N]:=0;let y[1]:=0;let y[N]:=0;let theta[1]:=0;let {i in {1,N}} v[i]:=0;let {i in {1,N}} phi[i]:=0;let {i in {1,N}} a[i]:=0;let {i in {1,N}} w[i]:=0;let {j in 1..D} cx[1,j]:=offset[j,1];let {j in 1..D} cy[1,j]:=offset[j,2];\n');
    fprintf(f,'printf "%%.17g %%.17g\\n",compound_cost,nominal_cost/weight_penalty+infeasibility >> "equivalence.txt";}\n');
    fprintf(f,'option solver "%s";option ipopt_options "max_iter=0 max_cpu_time=2 print_level=5 nlp_scaling_method=none linear_solver=ma97";solve;\n',strrep(runtime.ipopt,'\','/'));fclose(f);
    timer=tic;[status,log]=cp.RunAmpl(runtime,folder,'check.run',15);elapsed=toc(timer);assert(status==0,log);
    values=readmatrix(fullfile(folder,'equivalence.txt'),'FileType','text');assert(isequal(size(values),[8 2]));
    assert(all(abs(values(:,1)-values(:,2))<1e-11*max(1,abs(values(:,2)))));
    tokens=regexp(log,'Number of nonzeros in Lagrangian Hessian[. ]*:\s*(\d+)','tokens','once');assert(~isempty(tokens),log);
    records{variant}=struct('hessian_nonzeros',str2double(tokens{1}),'initialization_wall_s',elapsed, ...
        'objective_values',values,'maximum_objective_difference',max(abs(values(:,1)-values(:,2))));
end
assert(records{2}.hessian_nonzeros<records{1}.hessian_nonzeros);
assert(max(abs(records{1}.objective_values-records{2}.objective_values),[],'all')<1e-10);
report=struct('passed',true,'nodes',51,'discs',3,'aggregate',records{1},'local_sums',records{2});disp(report);
end
