function q=PathPose(pieces,s)
ends=[0 cumsum([pieces.length])];s=max(0,min(ends(end),s(:)));index=discretize(s,[-inf ends(2:end-1) inf]);q=zeros(numel(s),4);
for j=unique(index)'
 rows=index==j;parameter=lamiraux.Parameter(pieces(j),s(rows)-ends(j));q(rows,:)=lamiraux.Evaluate(pieces(j),parameter);
end
end
