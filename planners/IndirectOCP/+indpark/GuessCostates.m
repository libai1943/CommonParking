function z=GuessCostates(z,ctx)
% Least-squares recovery of the continuous adjoint/stationarity conditions.
% This is only a numerical initial guess, not a primal trajectory optimizer.
N=ctx.options.nodes;h=1/(N-1);Y=reshape(z(1:end-1),10,N);T=exp(z(end));q=Y(1:5,:);mid=(q(:,1:end-1)+q(:,2:end))/2;
u=diff(q(4:5,:),1,2)/(h*T);lim=[ctx.vehicle.amax;ctx.vehicle.wmax];u=max(-.9*lim,min(.9*lim,u));
[bu,gu]=indpark.Barrier(u,lim,ctx.mu);
[flow,D,H]=indpark.Kernel([mid;zeros(5,N-1)],ctx,ctx.reference);flow(4:5,:)=u;
H=H+sum(bu,1);A=D(1:5,1:5,:);rows=7*(N-1)+3;M=spalloc(rows,5*N,60*N);rhs=zeros(rows,1);
for k=1:N-1
 ix=(k-1)*5+(1:5);j=(k-1)*5+(1:5);Ad=A(:,:,k)';
 M(ix,j)=-eye(5)+.5*h*T*Ad;M(ix,j+5)=eye(5)+.5*h*T*Ad;rhs(ix)=h*T*flow(6:10,k);
 ir=5*(N-1)+(k-1)*2+(1:2);M(ir,j(4:5))=.5*eye(2);M(ir,j(4:5)+5)=.5*eye(2);rhs(ir)=-gu(:,k);
 M(end,j)=M(end,j)+.5*h*flow(1:5,k)';M(end,j+5)=M(end,j+5)+.5*h*flow(1:5,k)';
end
M(end-2,5)=1;M(end-1,5*N)=1;rhs(end)=-ctx.options.timeWeight-mean(H);
lambda=M\rhs;Y(6:10,:)=reshape(lambda,5,N);z=[Y(:);z(end)];
end
