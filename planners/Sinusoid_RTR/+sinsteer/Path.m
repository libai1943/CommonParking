function p=Path(phases,spacing)
index=sinsteer.Index(phases);z=index.segments;d=z(:,6);cuts=[1;find(diff(d)~=0)+1;size(z,1)+1];ends=[z(:,4);index.length];s=0;gear=zeros(0,1);cusps=1;
for j=1:numel(cuts)-1
 left=ends(cuts(j));right=ends(cuts(j+1));n=max(1,ceil((right-left)/spacing));h=linspace(left,right,n+1)';s=[s;h(2:end)];gear=[gear;repmat(d(cuts(j)),n,1)];cusps(end+1,1)=numel(s); %#ok<AGROW>
end
q=sinsteer.Sample(phases,index,s);p=struct('s',s,'x',q(:,1),'y',q(:,2),'theta',q(:,3),'phi',q(:,4),'gear',gear,'cusp_indices',cusps);
% Independent local charts can differ by full rotations; keep a continuous lift.
p.theta=unwrap(p.theta);
end
