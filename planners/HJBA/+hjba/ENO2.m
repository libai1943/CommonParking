function [minus,plus]=ENO2(a,h,dimension,periodic)
% Second-order one-sided ENO derivatives, with linear nonperiodic ghosts.
n=size(a,dimension);order=[dimension,setdiff(1:3,dimension,'stable')];b=permute(a,order);
if periodic,b=[b(end-1:end,:,:);b;b(1:2,:,:)];
else,b=[3*b(1,:,:)-2*b(2,:,:);2*b(1,:,:)-b(2,:,:);b;2*b(end,:,:)-b(end-1,:,:);3*b(end,:,:)-2*b(end-1,:,:)];end
left=(b(3:n+2,:,:)-b(2:n+1,:,:))/h;right=(b(4:n+3,:,:)-b(3:n+2,:,:))/h;
dl=(b(3:n+2,:,:)-2*b(2:n+1,:,:)+b(1:n,:,:))/h^2;
dc=(b(4:n+3,:,:)-2*b(3:n+2,:,:)+b(2:n+1,:,:))/h^2;
dr=(b(5:n+4,:,:)-2*b(4:n+3,:,:)+b(3:n+2,:,:))/h^2;
pick=abs(dl)<abs(dc);a2=dc;a2(pick)=dl(pick);minus=left+.5*h*a2;
pick=abs(dr)<abs(dc);a2=dc;a2(pick)=dr(pick);plus=right-.5*h*a2;
minus=ipermute(minus,order);plus=ipermute(plus,order);
end
