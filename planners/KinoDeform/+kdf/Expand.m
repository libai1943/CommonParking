function [tree,added]=Expand(tree,other,v,body,polygons,bbox,sgn,o)
added=false;
if rand<o.goalBias,q=other.q(randi(size(other.q,1)),:);else,q=[bbox(1)+rand*diff(bbox(1:2)),bbox(3)+rand*diff(bbox(3:4)),(2*rand-1)*pi,(2*rand-1)*v.vmax,(2*rand-1)*v.phimax];end
[~,near]=min(kdf.Distance(tree.q,q,o.scale));N=o.randomCommands;U=(2*rand(N,2)-1).*[v.amax,v.wmax];H=o.duration(1)+rand(N,1)*diff(o.duration);
steps=ceil(max(H)/o.treeStep);Q=zeros(N,5,steps+1);Q(:,:,1)=repmat(tree.q(near,:),N,1);valid=true(N,1);
for i=1:steps,Q(:,:,i+1)=kdf.Step(Q(:,:,i),U,H/steps,sgn,v.lw);end
endQ=Q(:,:,end);valid=valid & abs(endQ(:,4))<=v.vmax & abs(endQ(:,5))<=v.phimax & endQ(:,1)>=bbox(1) & endQ(:,1)<=bbox(2) & endQ(:,2)>=bbox(3) & endQ(:,2)<=bbox(4);
best=inf;choice=0;
for i=find(valid)'
 P=squeeze(Q(i,:,:))';if ~kdf.Free(P,linspace(0,H(i),steps+1)',U(i,:),sgn,body,polygons,v,o),continue;end
 neighbors=nnz(kdf.Distance(tree.q,endQ(i,:),o.scale)<o.neighbor);
 if neighbors<=best,best=neighbors;choice=i;end
end
if choice==0,return;end
tree.q(end+1,:)=endQ(choice,:);tree.parent(end+1,1)=near;tree.u(end+1,:)=U(choice,:);tree.duration(end+1,1)=H(choice);added=true;
end
