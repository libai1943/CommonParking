function library=BuildLatticePrimitives(outputFile)
%BUILDLATTICEPRIMITIVES Offline maneuver optimization; no scene is consulted.
% The same full model/cost is used later for graph search and improvement.
if nargin<1,outputFile=fullfile(tempdir,'CommonParking','LatticeOCP','primitive-library.mat');end
cfg=BenchmarkConfig();vehicle=cfg.vehicle;options=lattice.Config(vehicle);
options.maxCpuSeconds=120;options.barrierStrategy='adaptive'; % Offline subproblems have no collision constraints.
folder=fileparts(outputFile);if ~isfolder(folder),mkdir(folder);end
progressFile=[outputFile,'.progress.mat'];signature=jsonencode(struct('version',2,'vehicle',vehicle,'options',options));
base=cell(4,1);continuous=containers.Map('KeyType','char','ValueType','any');history=cell(0,1);clock=tic;
if isfile(progressFile)
    saved=load(progressFile);assert(strcmp(saved.signature,signature),'Offline cache parameters have changed.');
    base=saved.base;continuous=saved.continuous;history=saved.history;
end
for heading=1:4
    if ~isempty(base{heading}),continue;end
    theta=options.headings(heading);R=[cos(theta) -sin(theta);sin(theta) cos(theta)];collection=cell(15,1);at=0;
    for change=[-4:-1 1:4]
        target=mod(heading-1+change,16)+1;delta=atan2(sin(options.headings(target)-theta),cos(options.headings(target)-theta));
        at=at+1;collection{at}=make(delta,0,target,'heading');
    end
    for lateral=[-3:-1 1:3]
        at=at+1;collection{at}=make(0,lateral,heading,'parallel');
    end
    % The closest lattice-aligned straight segment is an exact analytic OCP
    % minimizer: all steering-related terms vanish and length is minimal.
    direction=[1 0;2 1;1 1;1 2];delta=direction(heading,:)*options.grid;len=norm(delta);
    fraction=linspace(0,1,options.primitiveIntervals+1)';states=[fraction*delta,repmat(theta,numel(fraction),1),zeros(numel(fraction),2)];
    collection{15}=struct('from',heading,'to',heading,'delta',delta,'gear',1,'length',len,'cost',len, ...
        'states',states,'controls',zeros(options.primitiveIntervals,1),'kind','straight');
    base{heading}=collection;
    save(progressFile,'base','continuous','history','signature','-v7');fprintf('Offline base heading %d/4 complete.\n',heading);
end
primitives=cell(480,1);at=0;
for quarter=0:3
    rotation=[cos(quarter*pi/2) -sin(quarter*pi/2);sin(quarter*pi/2) cos(quarter*pi/2)];
    for heading=1:4
        for m=1:15
            forward=base{heading}{m};forward.from=mod(forward.from-1+4*quarter,16)+1;forward.to=mod(forward.to-1+4*quarter,16)+1;
            forward.delta=round(forward.delta*rotation');forward.states(:,1:2)=forward.states(:,1:2)*rotation';forward.states(:,3)=forward.states(:,3)+quarter*pi/2;
            reverse=forward;reverse.delta=-forward.delta;reverse.states(:,1:2)=-forward.states(:,1:2);
            reverse.states(:,4:5)=-forward.states(:,4:5);reverse.controls=-forward.controls;reverse.gear=-1;
            at=at+1;primitives{at}=forward;at=at+1;primitives{at}=reverse;
        end
    end
end
library=struct('schema','CommonParking-LatticeOCP-primitives-1','vehicle',vehicle,'options',options,'primitives',{primitives}, ...
    'history',{history},'preparation_time_this_call_s',toc(clock),'signature',signature);
save(outputFile,'library','-v7');fprintf('Wrote %d primitives to %s\n',numel(primitives),outputFile);

    function primitive=make(delta,lateral,target,kind)
        key=sprintf('%.12g:%.12g',delta,lateral);
        if isKey(continuous,key),free=continuous(key);
        else
            problem=struct('start',zeros(5,1),'goal',[0;lateral;delta;0;0],'mode',1+(lateral~=0), ...
                'body',[0 0 1],'obstacles',zeros(0,3));
            initial=lattice.ManeuverSeed(delta,lateral,vehicle,options);
            [free,status]=lattice.Solve(problem,initial,vehicle,options);history{end+1}=compact(status,'free',key);
            assert(status.success,'Continuous offline maneuver failed: %s',key);continuous(key)=free;
        end
        finalWorld=free.states(end,1:2)*R';floorPoint=floor(finalWorld/options.grid);
        if lateral~=0
            % Round the requested lateral grid line, not the maneuver away:
            % otherwise a cheap straight motion could replace a side shift.
            directions=[1 0;2 1;1 1;1 2];d=directions(heading,:);normal=[-d(2) d(1)];
            level=round(lateral*norm(d)/options.grid);
            [a,b]=ndgrid(-8:8);gridPoints=floorPoint+[a(:) b(:)];
            candidates=gridPoints(gridPoints*normal'==level,:)*options.grid;
        else
            [a,b]=ndgrid(-1:2);candidates=(floorPoint+[a(:) b(:)])*options.grid;
        end
        [~,order]=sort(sum((candidates-finalWorld).^2,2));best=inf;winner=[];endpoint=[];
        for candidate=order(1:min(options.nearestEndpoints,numel(order)))'
            destination=candidates(candidate,:);local=destination*R;
            problem=struct('start',zeros(5,1),'goal',[local(:);delta;0;0],'mode',0,'body',[0 0 1],'obstacles',zeros(0,3));
            [solved,status]=lattice.Solve(problem,free,vehicle,options);history{end+1}=compact(status,'rounded',key);
            if status.success&&solved.objective<best,best=solved.objective;winner=solved;endpoint=destination;end
        end
        assert(~isempty(winner),'No valid rounded endpoint for heading %d and maneuver %s.',heading,key);
        z=winner.states;z(:,1:2)=z(:,1:2)*R';z(:,3)=z(:,3)+theta;
        primitive=struct('from',heading,'to',target,'delta',endpoint,'gear',1,'length',winner.lengths, ...
            'cost',best,'states',z,'controls',winner.controls,'kind',kind);
        fprintf('  Heading %d %s delta %.3f lateral %.1f: L %.3f, J %.3f\n',heading,kind,delta,lateral,winner.lengths,best);
    end
end
function item=compact(status,stage,key)
item=struct('stage',stage,'maneuver',key,'success',status.success,'native_status',status.solve_result_num);
if isfield(status,'objective'),item.objective=status.objective;item.shooting_residual=status.maximum_shooting_residual;end
end
