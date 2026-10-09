function J=Jacobian(z,r,b,env,o,pattern,colors)
J=spalloc(numel(r),numel(z),nnz(pattern));h=o.finiteDifferenceStep*(1+abs(z));
for color=1:max(colors)
 ids=find(colors==color);if isempty(ids),continue;end
 trial=z;trial(ids)=trial(ids)+h(ids);change=teb.Residual(trial,b,env,o)-r;
 [row,col]=find(pattern(:,ids));J=J+sparse(row,ids(col),change(row)./h(ids(col)),numel(r),numel(z));
end
end
