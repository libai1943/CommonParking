function [Q,t,E,controlIndex]=Rollout(q,U,duration,sgn,v,step)
% A branch is parameterized away from its fixed root; goal branch uses -f.
N=sum(ceil(duration/step));M=size(U,1);Q=zeros(N+1,5);Q(1,:)=q;t=zeros(N+1,1);controlIndex=zeros(N,1);E=[];
if nargout>2,E=zeros(5,2*M,N+1);end
n=1;
for j=1:M
 count=ceil(duration(j)/step);h=duration(j)/count;
 for z=1:count
  if nargout>2
   [Q(n+1,:),A,B]=kdf.Step(Q(n,:),U(j,:),h,sgn,v.lw);E(:,:,n+1)=A*E(:,:,n);E(:,2*j-1:2*j,n+1)=E(:,2*j-1:2*j,n+1)+B;
  else,Q(n+1,:)=kdf.Step(Q(n,:),U(j,:),h,sgn,v.lw);end
  t(n+1)=t(n)+h;controlIndex(n)=j;n=n+1;
 end
end
end
