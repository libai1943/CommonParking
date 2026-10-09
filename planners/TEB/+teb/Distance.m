function distance=Distance(q,env)
% Exact Euclidean body/polygon distance outside; negative SAT penetration inside.
n=size(q,1);v=env.vehicle;ct=cos(q(:,3));st=sin(q(:,3));
lo=[-v.lr,-v.lb/2];hi=[v.lw+v.lf,v.lb/2];mid=(lo+hi)/2;rad=(hi-lo)/2;
distance=zeros(n,numel(env.polygons));corners=[lo;hi(1) lo(2);hi;lo(1) hi(2)];
for j=1:numel(env.polygons)
 p=env.polygons{j};X=(p(:,1)'-q(:,1)).*ct+(p(:,2)'-q(:,2)).*st;
 Y=-(p(:,1)'-q(:,1)).*st+(p(:,2)'-q(:,2)).*ct;
 gap=max([min(X,[],2)-hi(1),lo(1)-max(X,[],2),min(Y,[],2)-hi(2),lo(2)-max(Y,[],2)],[],2);
 best=inf(n,1);
 for k=1:size(p,1)
  next=mod(k,size(p,1))+1;ex=X(:,next)-X(:,k);ey=Y(:,next)-Y(:,k);len=hypot(ex,ey);ax=-ey./len;ay=ex./len;
  proj=ax.*X+ay.*Y;center=ax*mid(1)+ay*mid(2);radius=abs(ax)*rad(1)+abs(ay)*rad(2);
  gap=max(gap,max(min(proj,[],2)-center-radius,center-radius-max(proj,[],2)));
  dx=max([lo(1)-X(:,k),X(:,k)-hi(1),zeros(n,1)],[],2);dy=max([lo(2)-Y(:,k),Y(:,k)-hi(2),zeros(n,1)],[],2);
  best=min(best,hypot(dx,dy));
  for corner=1:4
   dx=corners(corner,1)-X(:,k);dy=corners(corner,2)-Y(:,k);t=max(0,min(1,(dx.*ex+dy.*ey)./len.^2));
   best=min(best,hypot(dx-t.*ex,dy-t.*ey));
  end
 end
 best(gap<=0)=gap(gap<=0);distance(:,j)=best;
end
end
