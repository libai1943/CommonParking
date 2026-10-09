function [route,info]=Query(map,c,o)
clock=tic;route=struct([]);info=struct('success',false,'candidate_paths',0,'validated_edges',0,'rejected_edges',0,'time_s',0);
keep=map.curvature<=c.vehicle.kappa_max;ends=map.ends(keep,:);weights=map.weight(keep);primitives=map.primitives(keep);via=map.via(keep);q=map.nodes;
start=[c.task.x0 c.task.y0 c.task.theta0];goal=[c.task.xf c.task.yf c.task.thetaf];q=[q;start;goal];source=size(q,1)-1;target=size(q,1);connection=reedsSheppConnection('MinTurningRadius',c.vehicle.turning_radius_min,'ReverseCost',1);
for id=[source,target]
 distance=hypot(map.nodes(:,1)-q(id,1),map.nodes(:,2)-q(id,2));distance(~map.valid_nodes)=inf;[sorted,index]=sort(distance);index=index(isfinite(sorted));index=index(1:min(o.queryNeighbors,numel(index)));
 for near=index'
  pp=cprm.Shortest(connection,q(id,:),q(near,:),c.vehicle.kappa_max);
  ends(end+1,:)=[id near];weights(end+1,1)=sum(abs(pp(:,1)));primitives{end+1,1}=pp;via(end+1,1)=0; %#ok<AGROW>
 end
end
status=zeros(size(weights));ids=(1:numel(weights))';tableEdges=table(ends,weights,ids,'VariableNames',{'EndNodes','Weight','OriginalId'});graphMap=graph(tableEdges,table((1:size(q,1))','VariableNames',{'Index'}));
for iteration=1:o.queryIterations
 if toc(clock)>o.seconds,break;end
 [nodes,~,edges]=shortestpath(graphMap,source,target);if isempty(nodes),break;end
 info.candidate_paths=info.candidate_paths+1;original=graphMap.Edges.OriginalId(edges);bad=[];
 for id=original'
  if status(id)~=0,continue;end
  valid=cprm.ArcsFree(q(ends(id,1),:),primitives{id},c);info.validated_edges=info.validated_edges+1;
  if valid,status(id)=1;else,status(id)=-1;bad=id;info.rejected_edges=info.rejected_edges+1;break;end
 end
 if ~isempty(bad),graphMap=rmedge(graphMap,find(graphMap.Edges.OriginalId==bad));continue;end
 for j=1:numel(original)
  id=original(j);pp=primitives{id};if nodes(j)~=ends(id,1),pp=flipud(pp);pp(:,1)=-pp(:,1);end
  item=struct('start',q(nodes(j),:),'goal',q(nodes(j+1),:),'primitives',pp,'via',via(id),'control',zeros(0,2));
  if via(id)>0,item.control=map.control_points(via(id),:);end
  route=[route,item]; %#ok<AGROW>
 end
 info.success=true;break;
end
info.time_s=toc(clock);info.graph_nodes=numnodes(graphMap);info.graph_edges=numedges(graphMap);
end
