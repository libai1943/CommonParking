function [ineq,eq,GI,GE]=Constraints(z,ctx)
N=numel(ctx.grid);q=reshape(z(1:end-1),5,N);T=z(end);h=T*diff(ctx.grid);[f,A]=ppocp.Model(q,ctx.vehicle.lw);
qm=(q(:,1:end-1)+q(:,2:end))/2;qm(1:3,:)=qm(1:3,:)+(f(:,1:end-1)-f(:,2:end)).*h/8;[fm,Am]=ppocp.Model(qm,ctx.vehicle.lw);
eq=diff(q(1:3,:),1,2)-(f(:,1:end-1)+4*fm+f(:,2:end)).*h/6;eq=eq(:);
d=diff(q(4:5,:),1,2);lim=[ctx.vehicle.amax;ctx.vehicle.wmax];rate=[d-lim.*h;-d-lim.*h];[p,g]=ppocp.Potential(q(1:3,:),ctx.points,ctx.vehicle);ineq=[rate(:);p(:)];
if nargout<3,return;end
GE=zeros(numel(z),3*(N-1));GI=zeros(numel(z),4*(N-1)+N);E=[eye(3),zeros(3,2)];
for k=1:N-1
 ids=(k-1)*5+(1:5);ir=(k-1)*3+(1:3);Q0=.5*eye(5)+h(k)/8*[A(:,:,k);zeros(2,5)];Q1=.5*eye(5)-h(k)/8*[A(:,:,k+1);zeros(2,5)];
 GE(ids,ir)=(-E-h(k)/6*(A(:,:,k)+4*Am(:,:,k)*Q0))';GE(ids+5,ir)=(E-h(k)/6*(A(:,:,k+1)+4*Am(:,:,k)*Q1))';
 ds=ctx.grid(k+1)-ctx.grid(k);GE(end,ir)=(-ds/6*(f(:,k)+4*fm(:,k)+f(:,k+1))-h(k)/6*4*Am(:,1:3,k)*(ds/8*(f(:,k)-f(:,k+1))))';
 jr=(k-1)*4+(1:4);GI(ids(4:5),jr)=[-eye(2),eye(2)];GI(ids(4:5)+5,jr)=[eye(2),-eye(2)];GI(end,jr)=-ds*[lim;lim]';
end
for k=1:N,GI((k-1)*5+(1:3),4*(N-1)+k)=g(:,k);end
end
