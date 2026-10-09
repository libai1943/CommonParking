function report=TestLatticeOCP(libraryFile)
% Known feasible full-model warm starts, including a forward/reverse switch.
if nargin<1
    root=fileparts(fileparts(mfilename('fullpath')));
    libraryFile=fullfile(root,'planners','LatticeOCP','assets','primitive-library.mat');
end
stored=load(libraryFile,'library');library=stored.library;v=library.vehicle;
options=lattice.Config(v);primitives=library.primitives;
p=primitives{find(cellfun(@(x)x.from==1&&x.to==3&&x.gear==1,primitives),1)};
initial=struct('states',p.states,'controls',p.controls,'lengths',p.length, ...
    'phase',ones(numel(p.controls),1),'gear',p.gear);
discs=cp.CoveringDiscs(v,3,1);body=[discs.offsets,repmat(discs.radius,3,1)];
problem=struct('start',p.states(1,:)','goal',p.states(end,:)','mode',0,'body',body,'obstacles',zeros(0,3));
first=check(initial,p.cost);
next=primitives{find(cellfun(@(x)x.from==p.to&&x.to==5&&x.gear==-1,primitives),1)};
z=next.states;z(:,1:2)=z(:,1:2)+p.delta;
z(:,3)=z(:,3)+2*pi*round((p.states(end,3)-z(1,3))/(2*pi));
initial.states=[p.states;z(2:end,:)];initial.controls=[p.controls;next.controls];
initial.lengths=[p.length;next.length];initial.phase=[ones(numel(p.controls),1);2*ones(numel(next.controls),1)];
initial.gear=[1;-1];problem.goal=initial.states(end,:)';
second=check(initial,p.cost+next.cost);
report=struct('passed',true,'one_phase',first,'gear_change',second);disp(report);
    function item=check(seed,cost)
        [native,solver]=lattice.Solve(problem,seed,v,options);
        assert(solver.success,'A known-feasible lattice warm start did not converge.');
        assert(native.objective<=cost+1e-5,'Improvement increased the matched objective.');
        path=lattice.ExportPath(native,v,[0 0],eye(2),0);
        assert(norm([path.x(1);path.y(1);path.theta(1)]-problem.start(1:3))<1e-6);
        assert(norm([path.x(end);path.y(end);path.theta(end)]-problem.goal(1:3))<1e-6);
        for k=1:numel(path.cusp_indices)-1
            spacing=diff(path.s(path.cusp_indices(k):path.cusp_indices(k+1)));
            assert(max(spacing)-min(spacing)<1e-10&&max(spacing)<=.05+1e-10);
        end
        item=struct('native_status',solver.solve_result_num,'initial_cost',cost, ...
            'optimized_cost',native.objective,'maximum_shooting_residual',solver.maximum_shooting_residual, ...
            'gear_runs',numel(path.cusp_indices)-1);
    end
end
