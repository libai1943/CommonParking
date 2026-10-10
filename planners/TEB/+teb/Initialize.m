function b=Initialize(pose,o,origin)
pose(:,3)=unwrap(pose(:,3));d=diff(pose(:,1:2));a=diff(pose(:,3));
g=sign(sum(d.*[cos(pose(1:end-1,3)),sin(pose(1:end-1,3))],2));g(g==0)=1;
L=vecnorm(d,2,2);arc=L;turn=abs(a)>1e-8;arc(turn)=L(turn).*abs(a(turn)./(2*sin(a(turn)/2)));
dt=max(o.dt,arc);iv=g.*arc./dt;phi=atan(o.vehicle.lw*a./max(arc,1e-9).*g);
v=[0;.5*(iv(1:end-1)+iv(2:end));0];v([false;g(1:end-1).*g(2:end)<0;false])=0;
steer=[0;.5*(phi(1:end-1)+phi(2:end));0];
pose=[pose,v,steer];
b=struct('pose',pose,'dt',dt,'cost',Inf,'signature',NaN,'origin',origin,'solver',struct('success',false));
end
