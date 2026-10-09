function [candidates,report]=Explore(candidates,c,env,o)
start=[c.task.x0 c.task.y0];goal=[c.task.xf c.task.yf];delta=goal-start;L=norm(delta);
if L<1e-9,direction=[cos(c.task.theta0) sin(c.task.theta0)];else,direction=delta/L;end
normal=[-direction(2) direction(1)];points=start;trials=0;
while size(points,1)<o.samples+1&&trials<o.sampleTrials
 trials=trials+1;q=(start+goal)/2+(rand-.5)*max(1,L)*o.graphLengthScale*direction+(rand-.5)*o.graphWidth*normal;
 if teb.SegmentFree(q,q,env),points(end+1,:)=q;end %#ok<AGROW>
end
points=[points;goal];n=size(points,1);edges=false(n);
for k=1:n-1
 for j=2:n
  d=points(j,:)-points(k,:);
  if norm(d)>1e-10&&dot(d,direction)/norm(d)>o.forwardCosine&&teb.SegmentFree(points(k,:),points(j,:),env),edges(k,j)=true;end
 end
end
visits=0;found=0;initialCount=numel(candidates);walk(1);
report=struct('nodes',n,'edges',nnz(edges),'dfs_visits',visits,'sample_trials',trials,'goal_paths',found,'new_classes',numel(candidates)-initialCount);
 function walk(path)
  if numel(candidates)>=o.maximumClasses||visits>=o.maximumDfsVisits,return;end
  visits=visits+1;last=path(end);
  if edges(last,n)
   full=[path n];xy=points(full,:);signature=teb.Signature(xy,env);found=found+1;
   if isfinite(signature)&&(isempty(candidates)||all(abs([candidates.signature]-signature)>o.signatureTolerance))
    b=teb.Initialize(teb.Polyline(xy,c,o),o,'sampled_topology');b.signature=signature;candidates(end+1)=b; %#ok<AGROW>
   end
  end
  adjacent=find(edges(last,1:n-1));
  for next=adjacent
   if ~ismember(next,path),walk([path next]);end
   if numel(candidates)>=o.maximumClasses||visits>=o.maximumDfsVisits,return;end
  end
 end
end
