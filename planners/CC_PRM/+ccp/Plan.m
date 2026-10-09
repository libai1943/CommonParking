function result=Plan(c)
% CC-Steer (nine circle families plus topological path), generic PRM adapter.
ccp.EnsureNative();o=ccp.Config(c.vehicle);result=cp.EmptyResult('CC_PRM',c.id,'path');prior=rng;rng(o.seed+c.id);restore=onCleanup(@()rng(prior)); %#ok<NASGU>
v=c.vehicle;body=[v.lw+v.lf,v.lr,v.lb/2];ob=parking.PolygonData(c);t=c.task;
start=[t.x0,t.y0,t.theta0,0];goal=[t.xf,t.yf,t.thetaf,0];xy=[start(1:2);goal(1:2);vertcat(ob.vertices{:})];lower=min(xy)-o.padding;upper=max(xy)+o.padding;
q=[start;goal];A=zeros(0,1);B=A;weights=A;controls=cell(0,1);proposals=0;attempts=2;limits=[o.kappa,o.sigma];clock=tic;
[u,len]=cc_steer_mex('connect',[start;goal],[goal;start],limits);
for j=1:2,if ccp.Edge(q(j,:),u{j},ob.vertices,body,o),A(end+1,1)=j;B(end+1,1)=3-j;weights(end+1,1)=len(j);controls{end+1,1}=u{j};end,end
while size(q,1)<o.nodes&&toc(clock)<o.seconds
 sample=[lower+rand(1,2).*(upper-lower),2*pi*rand-pi,0];proposals=proposals+1;
 if cc_steer_mex('clearance',sample,body,ob.vertices,o.clearanceCap)<=o.margin+1e-10,continue;end
 difference=q(:,3)-sample(3);difference=atan2(sin(difference),cos(difference));metric=sum((q(:,1:2)-sample(1:2)).^2,2)+(difference/o.kappa).^2;
 [~,near]=mink(metric,min(o.neighbours,size(q,1)));n=numel(near);index=size(q,1)+1;
 [u,len]=cc_steer_mex('connect',[q(near,:);repmat(sample,n,1)],[repmat(sample,n,1);q(near,:)],limits);
 q(index,:)=sample;
 for k=1:2*n
  if k<=n,a=near(k);b=index;else,a=index;b=near(k-n);end
  attempts=attempts+1;if ccp.Edge(q(a,:),u{k},ob.vertices,body,o),A(end+1,1)=a;B(end+1,1)=b;weights(end+1,1)=len(k);controls{end+1,1}=u{k};end %#ok<AGROW>
 end
end
G=digraph(A,B,weights,size(q,1));[nodes,distance]=shortestpath(G,1,2,'Method','positive');
% MATLAB reorders graph edges; map each returned endpoint pair explicitly.
u=zeros(0,3);selected=zeros(0,1);
for k=1:numel(nodes)-1,edge=find(A==nodes(k)&B==nodes(k+1));assert(isscalar(edge));u=[u;controls{edge}];selected(end+1,1)=edge;end %#ok<AGROW>
result.solver=struct('success',isfinite(distance),'nodes',size(q,1),'edges',numel(A),'proposals',proposals,'connection_attempts',attempts,'search_time_s',toc(clock),'length_m',distance);
result.diagnostics=struct('options',o,'controls',u,'roadmap_nodes',q,'roadmap_edges',[A B weights],'selected_nodes',nodes,'selected_edges',selected,'start',start);
if ~isfinite(distance),result.status.code='roadmap_not_connected';result.status.message='No directed roadmap path within the disclosed finite sample/time budget.';return;end
result.path=ccp.Path(start,u,v,o.outputStep);e=[result.path.x(end),result.path.y(end),result.path.theta(end)]-goal(1:3);e(3)=atan2(sin(e(3)),cos(e(3)));assert(max(abs(e))<1e-7);
result.status=struct('success',true,'code','path_found','message','The directed roadmap connects exact poses with continuous-curvature CC-Steer and certified full-body edge separation.');
end
