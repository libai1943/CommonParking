function p=Path(q,u,v,spacing)
u=u(abs(u(:,1))>1e-12,:);d=sign(u(:,1));ends=[0;cumsum(abs(u(:,1)))];cuts=[1;find(diff(d)~=0)+1;size(u,1)+1];s=0;gear=zeros(0,1);cusps=1;
for j=1:numel(cuts)-1
 left=ends(cuts(j));right=ends(cuts(j+1));n=max(1,ceil((right-left)/spacing));z=linspace(left,right,n+1)';s=[s;z(2:end)];gear=[gear;repmat(d(cuts(j)),n,1)];cusps(end+1,1)=numel(s); %#ok<AGROW>
end
x=cc_steer_mex('sample',q,u,s);p=struct('s',s,'x',x(:,1),'y',x(:,2),'theta',x(:,3),'phi',atan(v.lw*x(:,4)),'gear',gear,'cusp_indices',cusps);
end
