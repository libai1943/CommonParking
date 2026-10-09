function selected=Slice(pieces,left,right)
selected=pieces([]);ends=[0 cumsum([pieces.length])];
for j=1:numel(pieces)
 lo=max(left,ends(j));hi=min(right,ends(j+1));if hi-lo<1e-10,continue;end
 p=pieces(j);
 if lo>ends(j)+1e-10||hi<ends(j+1)-1e-10
  p.range=lamiraux.Parameter(p,[lo hi]-ends(j))';p=lamiraux.Prepare(p);
 end
 selected(end+1)=p; %#ok<AGROW>
end
end
