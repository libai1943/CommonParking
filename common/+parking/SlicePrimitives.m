function part=SlicePrimitives(pp,a,b)
% Preserve the original exact arcs between two anchored arc-length events.
ends=[0;cumsum(abs(pp(:,1)))];part=zeros(0,2);
for i=1:size(pp,1)
 length0=min(b,ends(i+1))-max(a,ends(i));
 if length0>1e-11,part(end+1,:)=[sign(pp(i,1))*length0,pp(i,2)];end %#ok<AGROW>
end
end
