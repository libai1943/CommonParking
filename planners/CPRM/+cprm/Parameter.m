function t=Parameter(piece,s)
s=max(0,min(piece.length,s(:)));idx=discretize(s,[-inf;piece.table_s(2:end-1);inf]);
left=piece.table_t(idx);right=piece.table_t(idx+1);base=piece.table_s(idx);remaining=s-base;
t=left+(right-left).*remaining./(piece.table_s(idx+1)-base);
for iteration=1:5
 distance=cprm.IntegrateLength(piece,left,t);[~,speed]=cprm.Evaluate(piece,t);
 t=max(left,min(right,t+(remaining-distance)./speed));
end
t(s==0)=0;t(s==piece.length)=1;
end
