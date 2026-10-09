function report=TestWGRRT()
% Exercise candidate ordering, cyclic graph policy, reversal and Gaussian trees.
c=LoadCase(1);c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[30 31 31 30 30],'y',[30 30 31 31 30])};
o=wgrrt.Config();connection=reedsSheppConnection('MinTurningRadius',1/c.vehicle.kappa_max,'ForwardCost',1,'ReverseCost',1);
prior=rng;cleanup=onCleanup(@()rng(prior));rng(2017133); %#ok<NASGU>
a=[0 0 .2];b=[3 2 -.5];[~,costs]=connect(connection,a,b,'PathSegments','all');
[pp,ok,tested]=wgrrt.Connect(connection,a,b,c,o);assert(ok&&tested==1);
shortestError=abs(sum(abs(pp(:,1)))-min(costs));assert(shortestError<1e-10);
q=endpoint(a,pp);error=q-b;error(3)=atan2(sin(error(3)),cos(error(3)));assert(max(abs(error))<1e-9);
reverse=[-flipud(pp(:,1)),flipud(pp(:,2))];q=endpoint(q,reverse);error=q-a;error(3)=atan2(sin(error(3)),cos(error(3)));assert(max(abs(error))<1e-9);
% Both endpoint footprints are free, but every candidate crosses a thin wall.
cwall=c;cwall.obstacle.obs={struct('x',[5.99 6.01 6.01 5.99 5.99],'y',[-100 -100 100 100 -100])};
assert(parking.FootprintClearance([0 0 0;10 0 0],cwall,0));
[~,ok,tested]=wgrrt.Connect(connection,[0 0 0],[10 0 0],cwall,o);assert(~ok&&tested==3);
assert(~wgrrt.GeometricEdgeFree([0 0 0],[10 0 0],cwall,o));
% A cycle, a parallel edge and a two-node dead branch. Several stored edges run backwards.
graph=struct('q',[0 0 0;1 0 0;2 0 0;3 0 0;10 0 0;11 0 0], ...
 'from',[2;3;3;4;2;5;3],'to',[1;2;4;1;5;6;2], ...
 'cost',[1;1;1;6;9;1;3],'primitives',{{[-1 0];[-1 0];[1 0];[-4.5 0;1.5 0];[9 0];[1 0];[-2 0;1 0]}},'start',1,'goal',4);
[pp,value]=wgrrt.ValuePath(graph);assert(value.trimmed_nodes==2&&abs(value.length-3)<1e-12);
assert(isequal(value.path_nodes,[1;2;3;4]));assert(max(abs(endpoint([0 0 0],pp)-[3 0 0]))<1e-12);
% Two waypoint stages must preserve the previous graph and reach the last goal.
o.addedNodesPerStage=15;o.stageSampleLimit=3000;o.kinematicSeconds=30;
W=[0 0 0;2 1 .2;4 0 -.2];[graph,search]=wgrrt.Search(W,c,o,connection);assert(search.success);
assert(numel(search.stages)==2&&all(cellfun(@(s)s.new_nodes==15,search.stages)));
arcError=0;
for k=1:numel(graph.cost)
 q=endpoint(graph.q(graph.from(k),:),graph.primitives{k});error=q-graph.q(graph.to(k),:);error(3)=atan2(sin(error(3)),cos(error(3)));
 arcError=max(arcError,max(abs(error)));assert(wgrrt.ArcsFree(graph.q(graph.from(k),:),graph.primitives{k},c));
end
assert(arcError<1e-8);[pp,value]=wgrrt.ValuePath(graph);q=endpoint(W(1,:),pp);error=q-W(end,:);error(3)=atan2(sin(error(3)),cos(error(3)));assert(max(abs(error))<1e-8);
assert(max(abs(wgrrt.Metric(W,W(end,:),o)-wgrrt.Metric(W+[0 0 4*pi],W(end,:),o)))<1e-12);
report=struct('passed',true,'shortest_candidate_length_error',shortestError,'cyclic_value_length',3, ...
 'thin_wall_detected',true,'gaussian_graph_nodes',size(graph.q,1),'maximum_edge_pose_error',arcError,'gaussian_path_length',value.length);disp(report);
end
function q=endpoint(q,pp)
for k=1:size(pp,1),q=parking.IntegratePrimitive(q,pp(k,1),pp(k,2));end
end
