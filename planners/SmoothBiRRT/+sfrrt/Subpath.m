function arcs=Subpath(original,first,last)
ends=[0;cumsum(abs(original(:,1)))];arcs=zeros(0,2);
for j=1:size(original,1),length=max(0,min(last,ends(j+1))-max(first,ends(j)));if length>1e-10,arcs(end+1,:)=[sign(original(j,1))*length,original(j,2)];end;end %#ok<AGROW>
end
