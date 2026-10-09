function [pattern,colors]=Pattern(n,R)
% Graph coloring: pose edges touch at most 3 consecutive poses, time edges 2.
M=n-1;nv=3*(n-2)+M;nr=5*M+2*n+n*R;rows=[];cols=[];
for k=1:M
 add(k,[],k);add(M+k,[k k+1],[]);add(2*M+k,[k k+1],[]);
 add(3*M+k,[k k+1],k);add(4*M+k,[k k+1],k);
end
for k=1:n
 if k==1,pp=[1 2];tt=1;elseif k==n,pp=[n-1 n];tt=M;else,pp=k-1:k+1;tt=k-1:k;end
 add(5*M+k,pp,tt);add(5*M+n+k,pp,tt);
end
for j=1:R,for k=1:n,add(5*M+2*n+(j-1)*n+k,k,[]);end,end
pattern=sparse(rows,cols,true,nr,nv);colors=zeros(nv,1);
for k=2:n-1,colors(3*(k-2)+(1:3))=3*mod(k,3)+(1:3);end
colors(3*(n-2)+(1:M))=10+mod((1:M)',2);
 function add(row,poses,times)
  poses=reshape(poses(poses>1&poses<n),1,[]);cc=[reshape(3*(poses-2)+(1:3)',1,[]),3*(n-2)+times];
  rows=[rows,repmat(row,1,numel(cc))];cols=[cols,cc]; %#ok<AGROW>
 end
end
