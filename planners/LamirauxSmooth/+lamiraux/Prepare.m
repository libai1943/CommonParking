function piece=Prepare(piece)
last=inf;
for count=[16 32 64 128 256 512 1024 2048]
 t=linspace(piece.range(1),piece.range(2),count+1)';steps=lamiraux.IntegrateLength(piece,t(1:end-1),t(2:end));length=sum(steps);
 if abs(length-last)<1e-10*(1+length),break;end
 last=length;
end
assert(all(steps>0),'Nonregular or zero-length local curve.');piece.length=length;piece.table_t=t;piece.table_s=[0;cumsum(steps)];
end
