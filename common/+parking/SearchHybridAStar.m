function result=SearchHybridAStar(c,opt)
% Weighted hybrid A*: continuous poses, exact bicycle primitives, RS connection.
% Dolgov et al. (2010) (Dolgov et al.2010), Sec.2: shared ordinary Hybrid A* front end.
% Requires Navigation Toolbox; scene generation/plotting does not.
if nargin<2, opt=struct(); end
defaults=struct('xyResolution',0.2,'thetaResolution',pi/36,'step',0.5, ...
    'collisionStep',0.04,'clearance',0.12,'heuristicWeight',1.0, ...
    'switchPenalty',1.5,'reversePenalty',0.05,'maxExpanded',100000,'maxSeconds',180, ...
    'analyticEvery',5,'steeringSamples',3, ...
    'variableStep',true,'stepScale',0.1,'maximumStep',1.0);
keys=fieldnames(defaults);
for j=1:numel(keys), if ~isfield(opt,keys{j}),opt.(keys{j})=defaults.(keys{j});end,end
v=c.vehicle; t=c.task; start=[t.x0 t.y0 t.theta0]; goal=[t.xf t.yf t.thetaf];
assert(parking.FootprintClearance([start;goal],c,opt.clearance),'Start or goal is blocked.');
rs=reedsSheppConnection('MinTurningRadius',v.turning_radius_min,'ReverseCost',1);
allxy=[start(1:2);goal(1:2)];
for j=1:c.obstacle.num_obs, o=c.obstacle.obs{j};allxy=[allxy;o.x' o.y'];end %#ok<AGROW>
lower=min(allxy)-6; upper=max(allxy)+6; dims=ceil((upper-lower)/opt.xyResolution)+1;
setupClock=tic;H=parking.GridHeuristic(c,lower,upper,opt.xyResolution);setupTime=toc(setupClock);
if opt.variableStep
    field=parking.VoronoiField(c);setupTime=toc(setupClock);
end
nt=ceil(2*pi/opt.thetaResolution); best=sparse(prod(dims)*nt*2,1);
% Immutable node records avoid corrupting descendants when a cell improves.
cap=600000; Q=zeros(cap,3); G=inf(cap,1); parent=zeros(cap,1); prim=zeros(cap,2); dir=zeros(cap,1);
Q(1,:)=start; G(1)=0; count=1; expanded=0; open=1; priority=0;
finish=0; tail=[]; clock=tic;
while ~isempty(open) && expanded<opt.maxExpanded && toc(clock)<opt.maxSeconds
    [~,pos]=min(priority); cur=open(pos);open(pos)=[];priority(pos)=[];
    q0=Q(cur,:); key=index(q0,max(dir(cur),-1));
    if best(key)>0 && G(cur)>best(key)+1e-9,continue;end
    expanded=expanded+1;
    if expanded==1 || mod(expanded,opt.analyticEvery)==0
        [paths,costs]=connect(rs,q0,goal,'PathSegments','all');
        [~,order]=sort(costs(:));
        for ii=order'
            if ~isfinite(costs(ii)),continue;end
            p=paths{ii}; pp=fromRS(p,v.kappa_max);
            qq=parking.SamplePrimitives(q0,pp,opt.collisionStep);
            if parking.FootprintClearance(qq,c,opt.clearance)
                finish=cur;tail=pp;break;
            end
        end
        if finish>0,break;end
    end
    step=opt.step;
    if opt.variableStep
        obstacleDistance=parking.NearestObstacle(q0(1:2),c);
        if isempty(field.points),voronoiDistance=0;else,voronoiDistance=min(vecnorm(field.points-q0(1:2),2,2));end
        step=max(opt.xyResolution,min(opt.maximumStep,opt.stepScale*(obstacleDistance+voronoiDistance)));
    end
    for direction=[1 -1]
        for kap=linspace(-v.kappa_max,v.kappa_max,opt.steeringSamples)
            signed=direction*step;
            sample=parking.IntegratePrimitive(q0,linspace(0,signed,ceil(step/opt.collisionStep)+1),kap);
            q1=sample(end,:);
            if any(q1(1:2)<lower)||any(q1(1:2)>upper),continue;end
            ng=G(cur)+step*(1+opt.reversePenalty*(direction<0))+opt.switchPenalty*(dir(cur)~=0 && dir(cur)~=direction);
            key=index(q1,direction);
            if best(key)>0 && ng>=best(key)-1e-9,continue;end
            if ~parking.FootprintClearance(sample,c,opt.clearance),continue;end
            [~,h]=connect(rs,q1,goal);
            ij=round((q1(1:2)-H.lower)/H.ds)+1;
            if all(ij>=1)&&ij(1)<=H.nx&&ij(2)<=H.ny
                dh=H.distance(ij(1)+(ij(2)-1)*H.nx);
                if isfinite(dh),h(1)=max(h(1),dh);end
            end
            count=count+1;
            if count>cap,error('Search storage capacity exceeded.');end
            Q(count,:)=q1;G(count)=ng;parent(count)=cur;prim(count,:)=[signed kap];dir(count)=direction;
            best(key)=ng+eps;open(end+1)=count;priority(end+1)=ng+opt.heuristicWeight*h(1); %#ok<AGROW>
        end
    end
end
result=struct('success',finish>0,'expanded',expanded,'runtime',toc(clock),'options',opt);
result.heuristic_setup_time=setupTime;result.reference='Dolgov et al.2010, Dolgov et al. (2010), Sec.2';
result.search_bounds=[lower;upper];
if finish==0,result.primitives=zeros(0,2);return;end
p=[];cur=finish;
while parent(cur)>0,p=[prim(cur,:);p];cur=parent(cur);end %#ok<AGROW>
result.primitives=[p;tail];
[result.poses,result.direction,result.curvature,result.signed_step]=parking.SamplePrimitives(start,result.primitives,0.02);
result.path_length=sum(abs(result.primitives(:,1)));
result.gear_changes=sum(diff(result.direction)~=0);
    function id=index(q,d)
        ij=floor((q(1:2)-lower)/opt.xyResolution)+1;
        it=mod(round(mod(q(3),2*pi)/(2*pi)*nt),nt)+1;
        id=ij(1)+dims(1)*(ij(2)-1+dims(2)*(it-1+nt*(d<0)));
    end
end
function pp=fromRS(p,k)
kap=zeros(numel(p.MotionLengths),1);
for i=1:numel(kap)
    if strcmp(p.MotionTypes{i},'L'),kap(i)=k;end
    if strcmp(p.MotionTypes{i},'R'),kap(i)=-k;end
end
pp=[p.MotionLengths(:).*p.MotionDirections(:),kap];
pp(abs(pp(:,1))<1e-10,:)=[];
end
