function p=Output(edges,v,spacing)
% Each row stores its OWN reverse-search anchor; headings need not join exactly.
lengths=abs(edges(:,4));gear=-sign(edges(:,4));ends=[0;cumsum(lengths)];cuts=[1;find(diff(gear)~=0)+1;numel(gear)+1];
s=0;directions=zeros(0,1);cusps=1;
for j=1:numel(cuts)-1
 left=ends(cuts(j));right=ends(cuts(j+1));count=max(1,ceil((right-left)/spacing));nodes=linspace(left,right,count+1)';
 s=[s;nodes(2:end)];directions=[directions;repmat(gear(cuts(j)),count,1)];cusps(end+1,1)=numel(s); %#ok<AGROW>
end
q=zeros(numel(s),3);phi=zeros(numel(s),1);
for j=1:numel(s)
 edge=find(s(j)<=ends(2:end)+1e-10,1);local=max(0,min(lengths(edge),s(j)-ends(edge)));
 q(j,:)=cp.ArcPose(edges(edge,1:3),edges(edge,4:5),lengths(edge)-local);phi(j)=atan(v.lw*edges(edge,5));
end
p=struct('s',s,'x',q(:,1),'y',q(:,2),'theta',unwrap(q(:,3)),'phi',phi,'gear',directions,'cusp_indices',cusps);
end
