function [q,d,k,s] = SamplePrimitives(q0,primitives,ds)
% primitives: [signed_length, curvature]. Direction/curvature are per edge.
q=q0; d=zeros(0,1); k=d; s=d;
for j=1:size(primitives,1)
    len=primitives(j,1); kap=primitives(j,2);
    if abs(len)<1e-10, continue; end
    n=max(1,ceil(abs(len)/ds)); step=len/n;
    part=parking.IntegratePrimitive(q(end,:),step*(1:n),kap);
    q=[q;part]; d=[d;repmat(sign(len),n,1)]; %#ok<AGROW>
    k=[k;repmat(kap,n,1)]; s=[s;repmat(step,n,1)]; %#ok<AGROW>
end
end
