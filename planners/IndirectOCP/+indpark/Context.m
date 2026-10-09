function ctx=Context(c,o)
ctx.vehicle=c.vehicle;ctx.options=o;ctx.mu=o.barriers(1);ctx.mode='tracking';
a=-c.vehicle.lr;b=c.vehicle.lw+c.vehicle.lf;w=c.vehicle.lb/2;t=linspace(a,b,5);
ctx.points=[t(:),repmat(w,5,1);t(:),repmat(-w,5,1);a,0;b,0];
p=parking.PolygonData(c);ctx.rectangles=zeros(p.count,5);
for j=1:p.count
 q=p.vertices{j};assert(size(q,1)==4,'Only exact rectangle obstacles are supported.');
 e=q(2,:)-q(1,:);ang=atan2(e(2),e(1));R=[cos(ang),sin(ang);-sin(ang),cos(ang)];z=q*R';lo=min(z);hi=max(z);ctr=(lo+hi)/2*R;
 assert(abs(prod(hi-lo)-p.area(j))<1e-7,'Obstacle is not rectangular.');
 ctx.rectangles(j,:)=[ctr,ang,hi-lo];
end
ctx.start=[c.task.x0;c.task.y0;c.task.theta0;0];ctx.goal=[c.task.xf;c.task.yf;c.task.thetaf;0];
end
