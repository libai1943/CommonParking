function [r,J]=Residual(z,ctx)
N=ctx.options.nodes;h=1/(N-1);Y=reshape(z(1:end-1),10,N);T=exp(z(end));mid=(Y(:,1:end-1)+Y(:,2:end))/2;
[f,D,H,HG]=indpark.Kernel(mid,ctx,ctx.reference);dyn=diff(Y,1,2)-h*T*f;
b=[Y(1:4,1)-ctx.start;Y(1:4,end)-ctx.goal;Y(10,1);Y(10,end)];r=[dyn(:);b;ctx.options.timeWeight+mean(H)];
if nargout<2,return;end
neq=numel(z);I=zeros(211*(N-1)+10,1);C=I;V=I;at=0;
[rr,cc]=ndgrid(1:10,1:10);rr=rr(:);cc=cc(:);
for k=1:N-1
 row=(k-1)*10;left=-eye(10)-.5*h*T*D(:,:,k);right=eye(10)-.5*h*T*D(:,:,k);
 ids=at+(1:100);I(ids)=row+rr;C(ids)=row+cc;V(ids)=left(:);at=at+100;
 ids=at+(1:100);I(ids)=row+rr;C(ids)=row+10+cc;V(ids)=right(:);at=at+100;
 ids=at+(1:10);I(ids)=row+(1:10);C(ids)=neq;V(ids)=-h*T*f(:,k);at=at+10;
end
ids=at+(1:10);I(ids)=10*(N-1)+(1:10);C(ids)=[1:4,10*(N-1)+(1:4),10,10*N];V(ids)=1;at=at+10;
grad=zeros(10,N);grad(:,1:end-1)=.5*h*HG;grad(:,2:end)=grad(:,2:end)+.5*h*HG;
J=sparse([I(1:at);repmat(neq,10*N,1)],[C(1:at);(1:10*N)'],[V(1:at);grad(:)],neq,neq);
end
