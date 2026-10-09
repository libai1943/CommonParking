function [primitives,info]=ValuePath(graph)
% Leaf trimming, finite-graph value iteration, then Bellman policy recovery.
n=size(graph.q,1);active=true(n,1);degree=accumarray([graph.from;graph.to],1,[n 1]);
while true
 leaves=find(active&degree<=1);leaves=setdiff(leaves,[graph.start graph.goal]);if isempty(leaves),break;end
 active(leaves)=false;
 for node=leaves'
  edges=find(graph.from==node|graph.to==node);neighbors=graph.from(edges)+graph.to(edges)-node;
  degree=degree-accumarray(neighbors,1,[n 1]);
 end
end
edgeActive=active(graph.from)&active(graph.to);value=inf(n,1);value(graph.goal)=0;iterations=0;
for iteration=1:n
 before=value;iterations=iteration;
 for edge=find(edgeActive)'
  a=graph.from(edge);b=graph.to(edge);cost=graph.cost(edge);
  if a~=graph.goal,value(a)=min(value(a),cost+value(b));end
  if b~=graph.goal,value(b)=min(value(b),cost+value(a));end
 end
 if isequal(value,before),break;end
end
assert(isfinite(value(graph.start)),'The connected graph has no finite value path.');
node=graph.start;primitives=zeros(0,2);nodes=node;edgesUsed=[];
for count=1:n
 if node==graph.goal,break;end
 edges=find(edgeActive&(graph.from==node|graph.to==node));neighbors=graph.from(edges)+graph.to(edges)-node;
 [~,k]=min(graph.cost(edges)+value(neighbors));edge=edges(k);next=neighbors(k);pp=graph.primitives{edge};
 if graph.to(edge)==node,pp=[-flipud(pp(:,1)),flipud(pp(:,2))];end
 assert(value(next)<value(node),'The positive-cost Bellman policy must strictly descend.');
 primitives=[primitives;pp];edgesUsed(end+1,1)=edge;nodes(end+1,1)=next;node=next; %#ok<AGROW>
end
assert(node==graph.goal);info=struct('trimmed_nodes',nnz(~active),'value_iterations',iterations,'value',value, ...
 'path_nodes',nodes,'path_edges',edgesUsed,'length',sum(abs(primitives(:,1))));
assert(abs(info.length-value(graph.start))<1e-6*(1+info.length));
end
