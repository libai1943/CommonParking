function [pattern,colors]=Pattern(n,R)
M=n-1;nv=5*(n-2)+M;nr=6*M+2*n+(n+M)*R;rows=[];cols=[];
for k=1:M
 add(k,[],k);
 for j=1:3,add(M+(j-1)*M+k,[k k+1],k);end
 add(4*M+2*n+k,[k k+1],k);add(5*M+2*n+k,[k k+1],k);
end
for k=1:n,add(4*M+k,k,[]);add(4*M+n+k,k,[]);end
for j=1:R
 for k=1:n,add(6*M+2*n+(j-1)*n+k,k,[]);end
 for k=1:M,add(6*M+2*n+n*R+(j-1)*M+k,[k k+1],[]);end
end
pattern=sparse(rows,cols,true,nr,nv);colors=zeros(nv,1);
for k=2:n-1,colors(5*(k-2)+(1:5))=5*mod(k,2)+(1:5);end
colors(5*(n-2)+(1:M))=11;
 function add(row,poses,times)
 poses=reshape(poses(poses>1&poses<n),1,[]);cc=[reshape(5*(poses-2)+(1:5)',1,[]),5*(n-2)+times];
 rows=[rows,repmat(row,1,numel(cc))];cols=[cols,cc];
 end
end
