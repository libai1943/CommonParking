function qp=Prepare(z,c,options)
N=options.nodes;D=7*N+1;K=N-1;[~,H,f]=slifs.Objective(z,N);
lower=-inf(D,1);upper=inf(D,1);v=c.vehicle;
lower(3*N+(1:N))=-v.vmax;upper(3*N+(1:N))=v.vmax;
lower(4*N+(1:N))=-v.phimax;upper(4*N+(1:N))=v.phimax;
lower(end)=.1;upper(end)=options.maxTime;
for state=1:5
 index=(state-1)*N+[1 N];lower(index)=z(index);upper(index)=z(index);
end
% Rest-to-rest endpoint states, including zero steering.
for state=4:5,index=(state-1)*N+[1 N];lower(index)=0;upper(index)=0;end
i=(1:K)';difference=sparse([i;i],[i;i+1],[-ones(K,1);ones(K,1)],K,N);
A=sparse(4*K,D);A(1:K,3*N+(1:N))=difference;A(K+(1:K),3*N+(1:N))=-difference;
A(2*K+(1:K),4*N+(1:N))=difference;A(3*K+(1:K),4*N+(1:N))=-difference;
A(:,end)=-[repmat(v.amax/K,2*K,1);repmat(v.wmax/K,2*K,1)];
qp=struct('H',H,'f',f,'A',A,'b',zeros(4*K,1),'lower',lower,'upper',upper);
end
