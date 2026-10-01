function [nodes,D,weights,barycentric] = LegendreLobatto(N)
% N Legendre-Gauss-Lobatto nodes on [-1,1], exact degree N-1 differentiation.
assert(isscalar(N)&&N>=3&&N==fix(N));n=N-1;nodes=-cos(pi*(0:n)'/n);
for iteration=1:100
    old=nodes;P=zeros(N,N);P(:,1)=1;P(:,2)=nodes;
    for k=2:n,P(:,k+1)=((2*k-1)*nodes.*P(:,k)-(k-1)*P(:,k-1))/k;end
    nodes=old-(old.*P(:,N)-P(:,N-1))./(N*P(:,N));
    if max(abs(nodes-old))<4*eps,break;end
end
P=zeros(N,N);P(:,1)=1;P(:,2)=nodes;
for k=2:n,P(:,k+1)=((2*k-1)*nodes.*P(:,k)-(k-1)*P(:,k-1))/k;end
pn=P(:,N);delta=nodes-nodes';delta(1:N+1:end)=1;
D=(pn./pn')./delta;D(1:N+1:end)=0;D(1:N+1:end)=-sum(D,2);
weights=2./(n*N*pn.^2);barycentric=1./pn;
end
