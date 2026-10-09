function [value,H,f]=Objective(z,N)
% Eq. (1): squared state differences, not time-weighted accelerations.
q=reshape(z(1:7*N),N,7);value=sum(diff(q(:,4)).^2+diff(q(:,5)).^2)+z(end);
if nargout<2,return;end
K=N-1;difference=sparse([(1:K)';(1:K)'],[(1:K)';(2:N)'],[-ones(K,1);ones(K,1)],K,N);
H=sparse(7*N+1,7*N+1);block=2*(difference'*difference);
H(3*N+(1:N),3*N+(1:N))=block;H(4*N+(1:N),4*N+(1:N))=block;
f=zeros(7*N+1,1);f(end)=1;
end
