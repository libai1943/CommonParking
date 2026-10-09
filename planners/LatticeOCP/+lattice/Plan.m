function result=Plan(c)
% Complete-model lattice search followed by the paper's fixed-mode OCP.
lattice.EnsureNative();result=cp.EmptyResult('LatticeOCP',c.id,'path');options=lattice.Config(c.vehicle);
root=fileparts(fileparts(mfilename('fullpath')));folder=getenv('COMMONPARKING_LATTICE_DATA');if isempty(folder),folder=fullfile(root,'assets');end
stored=load(fullfile(folder,'primitive-library.mat'),'library');library=stored.library;
stored=load(fullfile(folder,'heuristic.mat'),'lookup');lookup=stored.lookup;
assert(isequaln(library.vehicle,c.vehicle)&&strcmp(lookup.primitive_signature,library.signature),'Offline data do not match the common vehicle or primitive library.');
[body,obstacles]=lattice.Circles(c);t=c.task;R=[cos(t.theta0) -sin(t.theta0);sin(t.theta0) cos(t.theta0)];origin=[t.x0 t.y0];
obstacles(:,1:2)=(obstacles(:,1:2)-origin)*R;goalXY=([t.xf t.yf]-origin)*R;goalTheta=atan2(sin(t.thetaf-t.theta0),cos(t.thetaf-t.theta0));
actual=[0 0 0;goalXY goalTheta];
if ~all(free(actual))
    result.status.code='endpoint_circle_cover_blocked';result.status.message='The paper three-circle vehicle/obstacle cover blocks a required task endpoint.';return;
end
% Remark 1: snap off-lattice endpoints and restore the exact pose in the OCP.
% Start is exactly on the lattice in a rigid local coordinate frame.
[gx,gy]=ndgrid(round(goalXY(1))+(-1:1),round(goalXY(2))+(-1:1));candidates=zeros(0,4);
for h=1:16
    heading=options.headings(h);error=atan2(sin(heading-goalTheta),cos(heading-goalTheta));
    if abs(error)>pi/4,continue;end
    q=[gx(:),gy(:),repmat(heading,numel(gx),1)];valid=free(q);
    merit=sum((q(:,1:2)-goalXY).^2,2)+(c.vehicle.turning_radius_min*error)^2;
    candidates=[candidates;gx(valid),gy(valid),repmat(h,nnz(valid),1),merit(valid)]; %#ok<AGROW>
end
if isempty(candidates),result.status.code='no_free_goal_lattice_state';result.status.message='No nearby free terminal lattice state.';return;end
[~,best]=min(candidates(:,4));goal=candidates(best,1:3);
edges=lattice.EdgeMatrix(library);swept=cell(size(library.primitives));
for j=1:numel(swept)
    p=library.primitives{j};s=linspace(0,p.length,ceil(p.length/options.collisionSpacing)+1)';z=lattice.SamplePrimitive(p,s,c.vehicle,options.gamma);
    x=z(:,1)+cos(z(:,3))*body(:,1)'-sin(z(:,3))*body(:,2)';y=z(:,2)+sin(z(:,3))*body(:,1)'+cos(z(:,3))*body(:,2)';
    radius=repmat(body(:,3)',size(z,1),1);swept{j}=[x(:),y(:),radius(:)];
end
allXY=[0 0;goal(1:2);obstacles(:,1:2)];lower=floor(min(allXY))-options.searchPadding;upper=ceil(max(allXY))+options.searchPadding;
[route,info]=lattice_graph_mex('search',edges,swept,obstacles,[0 0 1],goal,[lower(1) upper(1) lower(2) upper(2)],lookup.cost,16, ...
    [options.searchSeconds options.searchExpanded options.clearance]);
result.solver=struct('search_success',logical(info(1)),'expanded',info(2),'search_time_s',info(3),'lattice_cost',info(4));
result.diagnostics=struct('goal_lattice',goal,'goal_exact',[goalXY goalTheta],'lattice_route',route,'options',options);
if ~info(1)||isempty(route),result.status.code='lattice_search_failed';result.status.message='The specified primitive graph did not find a route within the resource bounds.';return;end
% Retain each primitive's shooting intervals: this keeps the full-model
% warm-start equalities, rather than introducing a new resampling defect.
initial=struct('states',zeros(0,5),'controls',zeros(0,1),'lengths',zeros(numel(route),1),'phase',zeros(0,1),'gear',zeros(numel(route),1));
position=[0 0];angle=0;
for m=1:numel(route)
    p=library.primitives{route(m)};z=p.states;z(:,1:2)=z(:,1:2)+position;z(:,3)=z(:,3)+2*pi*round((angle-z(1,3))/(2*pi));
    if m==1,initial.states=z;else,initial.states=[initial.states;z(2:end,:)];end %#ok<AGROW>
    initial.controls=[initial.controls;p.controls];initial.phase=[initial.phase;repmat(m,numel(p.controls),1)]; %#ok<AGROW>
    initial.lengths(m)=p.length;initial.gear(m)=p.gear;position=position+p.delta;angle=z(end,3);
end
goalTheta=goalTheta+2*pi*round((angle-goalTheta)/(2*pi));
problem=struct('start',zeros(5,1),'goal',[goalXY(:);goalTheta;0;0],'mode',0,'body',body,'obstacles',obstacles);
[native,solver]=lattice.Solve(problem,initial,c.vehicle,options);
search=result.solver;result.solver=solver;result.solver.search=search;result.solver.shooting_nodes=size(initial.states,1);
result.diagnostics.initial=initial;result.diagnostics.native=native;
if ~isempty(native)&&sum(native.lengths)>1e-8,result.path=lattice.ExportPath(native,c.vehicle,origin,R,t.theta0);end
if solver.success
    result.status=struct('success',true,'code','solved','message','Full-model lattice initialization and matched-cost multiphase OCP solved.');
else
    result.status.code='improvement_failed';result.status.message='The native fixed-mode OCP failed; the lattice warm-start is not relabelled as an optimized result.';
end
    function valid=free(q)
        x=q(:,1)+cos(q(:,3))*body(:,1)'-sin(q(:,3))*body(:,2)';y=q(:,2)+sin(q(:,3))*body(:,1)'+cos(q(:,3))*body(:,2)';
        valid=true(size(q,1),1);
        for o=1:size(obstacles,1),valid=valid&all((x-obstacles(o,1)).^2+(y-obstacles(o,2)).^2>(body(:,3)'+obstacles(o,3)+options.clearance).^2,2);end
    end
end
