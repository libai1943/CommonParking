function [f,J]=Model(q,L)
n=size(q,2);ct=cos(q(3,:));st=sin(q(3,:));v=q(4,:);p=q(5,:);f=[v.*ct;v.*st;v.*tan(p)/L];J=zeros(3,5,n);
J(1,3,:)=reshape(-v.*st,1,1,n);J(1,4,:)=reshape(ct,1,1,n);J(2,3,:)=reshape(v.*ct,1,1,n);J(2,4,:)=reshape(st,1,1,n);J(3,4,:)=reshape(tan(p)/L,1,1,n);J(3,5,:)=reshape(v./cos(p).^2/L,1,1,n);
end
