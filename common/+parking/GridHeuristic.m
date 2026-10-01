function H=GridHeuristic(c,lower,upper,ds)
% Dolgov et al. (2010) Sec.2.1: backward 2-D dynamic programming with obstacle geometry.
% Point occupancy is deliberately not a fixed-orientation inflated car map.
nx=ceil((upper(1)-lower(1))/ds)+1;ny=ceil((upper(2)-lower(2))/ds)+1;
[X,Y]=ndgrid(lower(1)+(0:nx-1)*ds,lower(2)+(0:ny-1)*ds);blocked=false(nx,ny);
for j=1:c.obstacle.num_obs,o=c.obstacle.obs{j};blocked=blocked|inpolygon(X,Y,o.x,o.y);end
ij=round(([c.task.xf c.task.yf]-lower)/ds)+1;goal=ij(1)+(ij(2)-1)*nx;
distance=inf(nx*ny,1);distance(goal)=0;closed=false(nx*ny,1);
keys=zeros(nx*ny*5,1);ids=keys;hn=0;push(goal,0);
dx=[-1 -1 -1 0 0 1 1 1];dy=[-1 0 1 -1 1 -1 0 1];cost=ds*hypot(dx,dy);
while hn>0
    [id,g]=pop();if closed(id),continue;end;closed(id)=true;
    x=mod(id-1,nx)+1;y=floor((id-1)/nx)+1;
    for k=1:8
        xx=x+dx(k);yy=y+dy(k);if xx<1||xx>nx||yy<1||yy>ny,continue;end
        child=xx+(yy-1)*nx;if blocked(child)||closed(child),continue;end
        ng=g+cost(k);if ng>=distance(child),continue;end
        distance(child)=ng;push(child,ng);
    end
end
H=struct('lower',lower,'ds',ds,'nx',nx,'ny',ny,'distance',distance);
    function push(id,key)
        hn=hn+1;i=hn;
        while i>1
            parent=floor(i/2);if keys(parent)<=key,break;end
            keys(i)=keys(parent);ids(i)=ids(parent);i=parent;
        end
        keys(i)=key;ids(i)=id;
    end
    function [id,key]=pop()
        id=ids(1);key=keys(1);lk=keys(hn);li=ids(hn);hn=hn-1;i=1;
        while 2*i<=hn
            child=2*i;if child<hn&&keys(child+1)<keys(child),child=child+1;end
            if keys(child)>=lk,break;end
            keys(i)=keys(child);ids(i)=ids(child);i=child;
        end
        if hn>0,keys(i)=lk;ids(i)=li;end
    end
end
