function [flow,D,ham,hgrad,u]=Kernel(y,ctx,reference)
% Canonical state/costate ODE derived before midpoint discretization.
n=size(y,2);q=y(1:5,:);l=y(6:10,:);v=ctx.vehicle;mu=ctx.mu;
[u,du,bu]=indpark.Control(l(4:5,:),[v.amax;v.wmax],mu);
ct=cos(q(3,:));st=sin(q(3,:));sp=1./cos(q(5,:)).^2;tp=tan(q(5,:));speed=q(4,:);
f=[speed.*ct;speed.*st;speed.*tp/v.lw;u];
g=zeros(5,n);H=zeros(5,5,n);
if strcmp(ctx.mode,'tracking')||strcmp(ctx.mode,'blend')
 e=q(1:3,:)-reference;weights=ctx.options.trackWeights;ell=sum(weights.*e.^2,1);g(1:3,:)=2*weights.*e;
 for a=1:3,H(a,a,:)=2*weights(a);end
 if strcmp(ctx.mode,'blend')
  alpha=ctx.blend;[ee,gg,HH]=indpark.Penalty(q(1:3,:),ctx);ell=(1-alpha)*ell+alpha*ee;g(1:3,:)=(1-alpha)*g(1:3,:)+alpha*gg;H(1:3,1:3,:)=(1-alpha)*H(1:3,1:3,:)+alpha*HH;
 end
else
 [ell,gg,HH]=indpark.Penalty(q(1:3,:),ctx);g(1:3,:)=gg;H(1:3,1:3,:)=HH;
end
[bv,bg,bh]=indpark.Barrier(q(4:5,:),[v.vmax;v.phimax],mu);ell=ell+sum(bv,1)+sum(bu,1);g(4:5,:)=bg;
H(4,4,:)=reshape(bh(1,:),1,1,n);H(5,5,:)=reshape(bh(2,:),1,1,n);
A=zeros(5,5,n);A(1,3,:)=reshape(-speed.*st,1,1,n);A(1,4,:)=reshape(ct,1,1,n);A(2,3,:)=reshape(speed.*ct,1,1,n);A(2,4,:)=reshape(st,1,1,n);
A(3,4,:)=reshape(tp/v.lw,1,1,n);A(3,5,:)=reshape(speed.*sp/v.lw,1,1,n);
g(3,:)=g(3,:)+speed.*(-l(1,:).*st+l(2,:).*ct);
g(4,:)=g(4,:)+l(1,:).*ct+l(2,:).*st+l(3,:).*tp/v.lw;
g(5,:)=g(5,:)+l(3,:).*speed.*sp/v.lw;
H(3,3,:)=H(3,3,:)+reshape(-speed.*(l(1,:).*ct+l(2,:).*st),1,1,n);
H(3,4,:)=H(3,4,:)+reshape(-l(1,:).*st+l(2,:).*ct,1,1,n);H(4,3,:)=H(3,4,:);
H(4,5,:)=H(4,5,:)+reshape(l(3,:).*sp/v.lw,1,1,n);H(5,4,:)=H(4,5,:);
H(5,5,:)=H(5,5,:)+reshape(2*l(3,:).*speed.*sp.*tp/v.lw,1,1,n);
flow=[f;-g];D=zeros(10,10,n);D(1:5,1:5,:)=A;D(6:10,6:10,:)=-permute(A,[2 1 3]);D(6:10,1:5,:)=-H;
D(4,9,:)=reshape(du(1,:),1,1,n);D(5,10,:)=reshape(du(2,:),1,1,n);
ham=ell+sum(l.*f,1);hgrad=[g;f];
end
