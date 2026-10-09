function [samples,report,grid,value,safe]=Guidance(c,o)
% Equations (2)-(7): a numerical BRT intersected with exact static safety.
p=parking.PolygonData(c);t=c.task;goal=[t.xf t.yf t.thetaf];start=[t.x0 t.y0 t.theta0];
points=[goal(1:2);start(1:2);vertcat(p.vertices{:})];bounds=[min(points)-o.padding;max(points)+o.padding];
n=o.gridPoints;grid=struct('x',linspace(bounds(1,1),bounds(2,1),n),'y',linspace(bounds(1,2),bounds(2,2),n),'theta',goal(3)+(-floor(n/2):floor(n/2))*(2*pi/n));
[X,Y]=ndgrid(grid.x,grid.y);D=atan2(sin(grid.theta-goal(3)),cos(grid.theta-goal(3)));
target=max(max(abs(X-goal(1))/o.targetXY,abs(Y-goal(2))/o.targetXY),reshape(abs(D)/o.targetHeading,1,1,[]))-1;
safe=false(n,n,n);for k=1:n,[~,gap]=parking.FootprintClearance([X(:),Y(:),repmat(grid.theta(k),n*n,1)],c,0);safe(:,:,k)=reshape(gap>0,n,n);end
% In the two explicitly parallel cases the final forward adjustment is
% preceded by a reverse approach. Other cases use one reverse-approach BRT.
parallel=ismember(c.id,[1 2]);speeds=-o.speed;if parallel,speeds=[o.speed,-o.speed];end
stages=cell(numel(speeds),1);value=target;
for k=1:numel(speeds)
 [value,stages{k}]=hjba.ReachTube(value,grid,speeds(k),o.speed*c.vehicle.kappa_max,o.horizon,o.cfl);
 if k<numel(speeds),value(~safe)=max(value(~safe),1);end
end
eligible=value<=0&safe&(X>=goal(1))&(Y>=goal(2));indices=find(eligible);
chosen=indices(randperm(numel(indices),min(o.connectedCount,numel(indices))));[ix,iy,it]=ind2sub(size(eligible),chosen);
samples=[grid.x(ix)',grid.y(iy)',grid.theta(it)'];
report=struct('bounds',bounds,'grid_size',[n n n],'target_halfwidth',[o.targetXY o.targetXY o.targetHeading], ...
 'stages',{stages},'safe_count',nnz(safe),'eligible_count',numel(indices),'samples',samples,'sample_indices',chosen,'sample_values',value(chosen),'sample_safe',safe(chosen));
end
