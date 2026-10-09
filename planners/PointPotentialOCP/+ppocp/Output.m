function [trajectory,native]=Output(z,v,frame,o)
N=(numel(z)-1)/5;states=reshape(z(1:end-1),5,N);knots=linspace(0,z(end),N);cusps=[];
for k=1:N-1
    if states(4,k)*states(4,k+1)<0
        tc=knots(k)-states(4,k)*(knots(k+1)-knots(k))/(states(4,k+1)-states(4,k));
        if tc>knots(k)&&tc<knots(k+1),cusps(end+1)=tc;end %#ok<AGROW>
    end
end
regular=linspace(0,z(end),ceil(z(end)/o.outputStep)+1);
raw=[regular,knots,cusps];priority=[ones(size(regular)),2*ones(size(knots)),3*ones(size(cusps))];
priority(raw==0|raw==z(end))=4;
[raw,order]=sort(raw);priority=priority(order);keep=true(size(raw));anchor=1;
for k=2:numel(raw)
    if raw(k)-raw(anchor)<=32*eps(max(1,z(end)))
        if priority(k)>priority(anchor),keep(anchor)=false;anchor=k;else,keep(k)=false;end
    else,anchor=k;end
end
times=raw(keep);[q,d]=ppocp.Dense(z,v,times);
[found,ix]=ismember(cusps,times);
for k=find(~found)
    [gap,ix(k)]=min(abs(times-cusps(k)));assert(gap<=32*eps(max(1,z(end))));
end
ix=unique(ix);q(4,ix)=0;xy=q(1:2,:)'*frame.R+frame.origin;
trajectory=struct('t',times','x',xy(:,1),'y',xy(:,2),'theta',q(3,:)'+frame.angle, ...
    'v',q(4,:)','phi',q(5,:)','a',d(4,:)','omega',d(5,:)');
native=struct('z',z,'local_states',states','t',knots','frame',frame,'cusp_indices',ix(:));
end
