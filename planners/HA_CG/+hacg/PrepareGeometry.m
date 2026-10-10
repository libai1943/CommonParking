function geometry=PrepareGeometry(c)
% Cache only static polygon data for this call, never search answers.
polygons=cell(c.obstacle.num_obs,1);starts=zeros(0,2);edges=starts;
for j=1:c.obstacle.num_obs
 o=c.obstacle.obs{j};p=[o.x(1:end-1)' o.y(1:end-1)'];e=p([2:end 1],:)-p;
 axes=[-e(:,2),e(:,1)];axes=axes./vecnorm(axes,2,2);lo=zeros(size(axes,1),1);hi=lo;
 for k=1:size(axes,1),projection=p*axes(k,:)';lo(k)=min(projection);hi(k)=max(projection);end
 polygons{j}=struct('p',p,'axes',axes,'lo',lo,'hi',hi);starts=[starts;p];edges=[edges;e]; %#ok<AGROW>
end
geometry=struct('vehicle',c.vehicle,'polygons',{polygons},'starts',starts,'edges',edges,'squares',sum(edges.^2,2));
end
