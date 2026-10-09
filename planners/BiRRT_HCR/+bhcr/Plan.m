function result=Plan(c)
% Bidirectional RRT* with the HCR00 cubic-spiral extension, not an HA* initializer.
bhcr.EnsureNative();options=bhcr.Config(c.vehicle);result=cp.EmptyResult('BiRRT_HCR',c.id,'path');
previous=rng;restore=onCleanup(@()rng(previous));rng(options.seed+c.id,'twister');
polygons=parking.PolygonData(c);v=c.vehicle;body=[v.lw+v.lf v.lr v.lb/2];
limits=[options.kappa options.sigma options.rho];cap=options.hardClearance+options.inflationRadius;
t=c.task;start=[t.x0 t.y0 t.theta0 0];goal=[t.xf t.yf t.thetaf 0];
xy=[start(1:2);goal(1:2);vertcat(polygons.vertices{:})];lower=min(xy)-options.samplePadding;upper=max(xy)+options.samplePadding;
if any(clearance([start;goal])<=options.hardClearance)
    result.status.code='endpoint_clearance';result.status.message='A required endpoint violates the paper 0.1 m hard clearance.';return;
end
grid=bhcr.FootprintGrid(c,options);
emptyStats=[0 0 0 0 0 0];trees={newTree(start),newTree(goal)};
bridges=struct('a',{},'b',{},'controls',{},'stats',{});best=inf;bestControls=zeros(0,4);firstSolution=NaN;rewires=0;iteration=0;clock=tic;
% A direct root connection is the initial bidirectional connection attempt.
[u,~]=hcr_steer_mex('connect',start,goal,limits);[valid,stats]=edge(start,u{1});
if valid,bridges(1)=struct('a',1,'b',1,'controls',u{1},'stats',stats);updateBest();end
while withinBudget()&&iteration<options.maxIterations
    iteration=iteration+1;side=1+mod(iteration-1,2);other=3-side;T=trees{side};U=trees{other};
    if rand<options.goalBias,sample=U.q(1,:);sample(4)=0;
    else,sample=[lower+rand(1,2).*(upper-lower),2*pi*rand-pi,0];end
    if clearance(sample)<=options.hardClearance,continue;end
    [candidate,lengths]=hcr_steer_mex('connect',T.q,sample,limits);
    [minimum,nearest]=min(lengths);if minimum<1e-6,continue;end
    radius=options.gamma*(log(size(T.q,1)+1)/(size(T.q,1)+1))^(1/3);
    near=unique([find(lengths<=radius);nearest]);parent=0;parentCost=inf;parentStats=[];edgeStats=[];
    % Compare COMPLETE path costs: normalized curvature and footprint cost
    % are not additive edge costs; see README for the chosen coefficients.
    for node=near'
        [valid,e]=edge(T.q(node,:),candidate{node});if ~valid,continue;end
        s=append(T.stats(node,:),e);value=cost(s);
        if value<parentCost,parent=node;parentCost=value;parentStats=s;edgeStats=e;end
    end
    if parent==0,continue;end
    current=size(T.q,1)+1;T.q(current,:)=sample;T.parent(current,1)=parent;
    T.controls{current,1}=candidate{parent};T.edgeStats(current,:)=edgeStats;T.stats(current,:)=parentStats;
    % Rewire only non-ancestors; all sampled nodes have zero curvature/rate.
    ancestors=current;at=parent;while at>0,ancestors(end+1)=at;at=T.parent(at);end %#ok<AGROW>
    for node=near'
        if ismember(node,ancestors),continue;end
        reverse=bhcr.Reverse(candidate{node});[valid,e]=edge(sample,reverse);if ~valid,continue;end
        s=append(T.stats(current,:),e);
        if cost(s)+1e-10<cost(T.stats(node,:))
            T.parent(node)=current;T.controls{node}=reverse;T.edgeStats(node,:)=e;T.stats(node,:)=s;rewires=rewires+1;
            queue=node;
            while ~isempty(queue)
                at=queue(1);queue(1)=[];children=find(T.parent==at)';
                for child=children,T.stats(child,:)=append(T.stats(at,:),T.edgeStats(child,:));end
                queue=[queue children]; %#ok<AGROW>
            end
        end
    end
    trees{side}=T;
    % Connect the newly inserted node to the opposite tree's near set.
    [candidate,lengths]=hcr_steer_mex('connect',U.q,sample,limits);[~,nearest]=min(lengths);
    radius=options.gamma*(log(size(U.q,1)+1)/(size(U.q,1)+1))^(1/3);near=unique([find(lengths<=radius);nearest]);
    for node=near'
        [valid,e]=edge(U.q(node,:),candidate{node});if ~valid,continue;end
        if side==1
            bridge=struct('a',current,'b',node,'controls',bhcr.Reverse(candidate{node}),'stats',reverseStats(e));
        else
            bridge=struct('a',node,'b',current,'controls',candidate{node},'stats',e);
        end
        bridges(end+1)=bridge; %#ok<AGROW>
    end
    updateBest();
end
result.solver=struct('success',isfinite(best),'iterations',iteration,'nodes',[size(trees{1}.q,1),size(trees{2}.q,1)], ...
    'rewires',rewires,'bridges',numel(bridges),'time_to_first_solution_s',firstSolution,'search_time_s',toc(clock),'cost',best, ...
    'seed',options.seed+c.id);
result.diagnostics=struct('options',options,'sample_bounds',[lower;upper],'controls',bestControls,'start',start,'grid',grid);
if isfinite(best)
    cfg=BenchmarkConfig();result.path=bhcr.Path(start,bestControls,v,cfg.output.path_spacing_max_m);
    q=[result.path.x(end),result.path.y(end),result.path.theta(end)];difference=q-goal(1:3);difference(3)=atan2(sin(difference(3)),cos(difference(3)));
    assert(max(abs(difference))<1e-5,'HCR-Steer path endpoint mismatch.');
    result.status=struct('success',true,'code','solved','message','Bidirectional RRT* found a collision-checked HCR00-Steer path within its search budget.');
else
    result.status.code='search_budget_exhausted';result.status.message='No connecting path found within the paper five-second initial-solution budget.';
end

    function yes=withinBudget()
        if isnan(firstSolution),yes=toc(clock)<options.firstSolutionSeconds;
        else,yes=toc(clock)<firstSolution+options.refinementSeconds;end
    end
    function T=newTree(q)
        T=struct('q',q,'parent',0,'controls',{{zeros(0,4)}},'edgeStats',emptyStats,'stats',emptyStats);
    end
    function gap=clearance(q)
        gap=hcr_steer_mex('clearance',q,body,polygons.vertices,cap);
    end
    function [valid,s]=edge(q,u)
        if isempty(u),valid=true;s=emptyStats;return;end
        length=sum(abs(u(:,1)));ends=[0;cumsum(abs(u(:,1)))];
        arc=unique([linspace(0,length,max(1,ceil(length/options.collisionStep))+1)';ends]);
        poses=hcr_steer_mex('sample',q,u,arc);gap=min(clearance(poses));valid=gap>options.hardClearance;
        if ~valid,s=emptyStats;return;end
        integral=bhcr.CurvatureIntegral(u);
        footprint=bhcr.FootprintCost(poses,grid,body);
        exposure=trapz(arc,footprint);
        d=sign(u(:,1));s=[length,nnz(diff(d)),integral,exposure,d(1),d(end)];
    end
    function s=append(a,b)
        if a(1)<1e-12,s=b;return;elseif b(1)<1e-12,s=a;return;end
        s=[a(1)+b(1),a(2)+b(2)+(a(6)~=b(5)),a(3)+b(3),a(4)+b(4),a(5),b(6)];
    end
    function s=reverseStats(s)
        gears=s(5:6);s(5:6)=-fliplr(gears);
    end
    function value=cost(s)
        value=options.weights*[s(1);s(2);s(3)/(options.kappa*max(s(1),eps));s(4)/max(s(1),eps)];
    end
    function controls=branch(T,node)
        controls=zeros(0,4);
        while node>1,controls=[T.controls{node};controls];node=T.parent(node);end %#ok<AGROW>
    end
    function updateBest()
        if isnan(firstSolution)&&toc(clock)>options.firstSolutionSeconds,return;end
        for connection=1:numel(bridges)
            b=bridges(connection);s=append(append(trees{1}.stats(b.a,:),b.stats),reverseStats(trees{2}.stats(b.b,:)));value=cost(s);
            if value+1e-10<best
                best=value;bestControls=[branch(trees{1},b.a);b.controls;bhcr.Reverse(branch(trees{2},b.b))];
                if isnan(firstSolution),firstSolution=toc(clock);end
            end
        end
    end
end
