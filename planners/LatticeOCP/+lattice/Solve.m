function [native,solver]=Solve(problem,initial,vehicle,options)
% One common OCP for offline primitives and the online multiphase improvement.
runtime=cp.AmplRuntime();folder=cp.RuntimeDirectory();
root=fileparts(fileparts(mfilename('fullpath')));copyfile(fullfile(root,'model.mod'),folder);
E=size(initial.states,1)-1;P=numel(initial.lengths);B=size(problem.body,1);O=size(problem.obstacles,1);
counts=accumarray(initial.phase(:),1,[P 1]);
f=fopen(fullfile(folder,'data.dat'),'w');closer=onCleanup(@()fclose(f));
names={'E','P','B','O','lw','phimax','rate_max','acceleration_max','gamma','max_length','terminal_mode','clearance'};
values=[E P B O vehicle.lw vehicle.phimax options.steeringRatePerMetre options.steeringAccelerationPerMetre2 options.gamma options.maxLength problem.mode options.clearance];
for j=1:numel(names),fprintf(f,'param %s:=%.17g;\n',names{j},values(j));end
vector(f,'phase',initial.phase);vector(f,'num_steps',counts);vector(f,'gear',initial.gear);vector(f,'start',problem.start);vector(f,'goal',problem.goal);
matrix(f,'body',problem.body);matrix(f,'obstacle',problem.obstacles);clear closer;
f=fopen(fullfile(folder,'initial.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'let seg_length[%d]:=%.17g;\n',[(1:P)' initial.lengths(:)]');
for state=1:5,fprintf(f,['let z[%d,',num2str(state),']:=%.17g;\n'],[(0:E)' initial.states(:,state)]');end
fprintf(f,'let u[%d]:=%.17g;\n',[(1:E)' initial.controls(:)]');clear closer;
f=fopen(fullfile(folder,'solve.run'),'w');closer=onCleanup(@()fclose(f));
fprintf(f,'reset;model model.mod;data data.dat;include initial.run;\noption solver "%s";\n',strrep(runtime.ipopt,'\','/'));
fprintf(f,'option ipopt_options "tol=%.12g acceptable_tol=%.12g max_iter=%d max_cpu_time=%.9g print_level=3 mu_strategy=%s linear_solver=ma97";\n',options.tolerance,options.tolerance,options.maxIterations,options.maxCpuSeconds,options.barrierStrategy);
fprintf(f,'solve;\nprintf "%%d\\n%%s\\n%%s\\n",solve_result_num,solve_result,solve_message > "status.txt";\n');
fprintf(f,'printf "%%.17g\\n",matched_cost > "cost.txt";\nprintf {p in 1..P} "%%.17g\\n",seg_length[p] > "lengths.txt";\n');
fprintf(f,'printf {i in 0..E} "%%.17g %%.17g %%.17g %%.17g %%.17g\\n",z[i,1],z[i,2],z[i,3],z[i,4],z[i,5] > "states.txt";\n');
fprintf(f,'printf {e in 1..E} "%%.17g\\n",u[e] > "controls.txt";\n');clear closer;
[process,log]=cp.RunAmpl(runtime,folder,'solve.run',2*options.maxCpuSeconds+30);
f=fopen(fullfile(folder,'solver.log'),'w');fprintf(f,'%s',log);fclose(f);
solver=struct('success',false,'process_exit_code',process,'solve_result_num',NaN,'message',log,'working_directory',folder);native=[];
if isfile(fullfile(folder,'status.txt'))
    lines=splitlines(string(fileread(fullfile(folder,'status.txt'))));solver.solve_result_num=str2double(lines(1));solver.message=char(join(lines(2:end),newline));
    solver.success=process==0&&solver.solve_result_num>=0&&solver.solve_result_num<100&&strcmp(lines(2),'solved');
end
files={'states.txt','controls.txt','lengths.txt','cost.txt'};
if ~all(cellfun(@(x)isfile(fullfile(folder,x)),files)),solver.success=false;return;end
z=readmatrix(fullfile(folder,files{1}),'FileType','text');u=readmatrix(fullfile(folder,files{2}),'FileType','text');lengths=readmatrix(fullfile(folder,files{3}),'FileType','text');cost=readmatrix(fullfile(folder,files{4}),'FileType','text');
if ~isequal(size(z),[E+1 5])||numel(u)~=E||numel(lengths)~=P||~isscalar(cost)||~isreal([z(:);u(:);lengths(:);cost])||any(~isfinite([z(:);u(:);lengths(:);cost]))||any(lengths<0)
    solver.success=false;solver.message='Invalid OCP output.';return;
end
h=lengths(initial.phase)./counts(initial.phase);gears=initial.gear(:);[predicted,stageCost]=lattice.Step(z(1:end-1,:),u(:),h,gears(initial.phase),vehicle,options.gamma);
solver.maximum_shooting_residual=max(abs(predicted-z(2:end,:)),[],'all');solver.objective=cost;solver.objective_recomputation_error=abs(cost-sum(stageCost));
solver.success=solver.success&&solver.maximum_shooting_residual<1e-6&&solver.objective_recomputation_error<1e-6;
native=struct('states',z,'controls',u(:),'lengths',lengths(:),'phase',initial.phase(:),'gear',initial.gear(:),'objective',cost);
end
function vector(f,name,value)
fprintf(f,'param %s:=\n',name);fprintf(f,'%d %.17g\n',[(1:numel(value))' value(:)]');fprintf(f,';\n');
end
function matrix(f,name,value)
fprintf(f,'param %s: 1 2 3 :=\n',name);fprintf(f,'%d %.17g %.17g %.17g\n',[(1:size(value,1))' value]');fprintf(f,';\n');
end
