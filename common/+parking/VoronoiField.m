function field=VoronoiField(c,ds)
% Grid approximation to polygon GVD; no lane graph and no artificial obstacles.
if nargin<2,ds=0.20;end
P=[];for j=1:c.obstacle.num_obs,o=c.obstacle.obs{j};P=[P;o.x' o.y'];end %#ok<AGROW>
lo=min(P)-4;hi=max(P)+4;[X,Y]=ndgrid(lo(1):ds:hi(1),lo(2):ds:hi(2));
[d,~,label]=parking.NearestObstacle([X(:) Y(:)],c);D=reshape(d,size(X));L=reshape(label,size(X));
bx=L(1:end-1,:)~=L(2:end,:)&D(1:end-1,:)>0.05&D(2:end,:)>0.05;
by=L(:,1:end-1)~=L(:,2:end)&D(:,1:end-1)>0.05&D(:,2:end)>0.05;
xx=(X(1:end-1,:)+X(2:end,:))/2;yy=(Y(1:end-1,:)+Y(2:end,:))/2;P=[xx(bx),yy(bx)];
xx=(X(:,1:end-1)+X(:,2:end))/2;yy=(Y(:,1:end-1)+Y(:,2:end))/2;P=[P;xx(by),yy(by)];
field=struct('points',unique(P,'rows'),'resolution',ds);
end
