function p=Prepare(p)
if strcmp(p.type,'arc'),p.table_t=[0;1];p.table_s=[0;p.length];return;end
last=inf;
for count=[16 32 64 128 256 512 1024 2048]
 t=linspace(0,1,count+1)';steps=cprm.IntegrateLength(p,t(1:end-1),t(2:end));length=sum(steps);
 if abs(length-last)<1e-10*(1+length),break;end
 last=length;
end
assert(all(steps>0));p.length=length;p.table_t=t;p.table_s=[0;cumsum(steps)];
end
