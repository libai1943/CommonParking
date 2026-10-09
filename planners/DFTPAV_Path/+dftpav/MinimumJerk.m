function [C,system]=MinimumJerk(q,p0,p1,v0,v1,h)
% Normalized local time u in [0,1]. C4 joins give the minimum-jerk solution.
M=size(q,1)+1;A=zeros(6*M);B=zeros(6*M,2);row=0;waypointRows=zeros(M-1,1);
for d=0:2,row=row+1;A(row,1:6)=dftpav.Basis(0,d);end
B(1,:)=p0;B(2,:)=h*v0;
for j=1:M-1
 ids=(j-1)*6+(1:6);next=ids+6;row=row+1;A(row,ids)=dftpav.Basis(1,0);B(row,:)=q(j,:);waypointRows(j)=row;
 for d=0:4,row=row+1;A(row,ids)=dftpav.Basis(1,d);A(row,next)=-dftpav.Basis(0,d);end
end
for d=0:2,row=row+1;A(row,end-5:end)=dftpav.Basis(1,d);end
B(end-2,:)=p1;B(end-1,:)=h*v1;
C=A\B;Q=zeros(6);for i=3:5,for j=3:5,Q(i+1,j+1)=prod(i-2:i)*prod(j-2:j)/(i+j-5);end,end
system=struct('A',A,'B',B,'Q',Q,'waypoint_rows',waypointRows,'start_rows',1:3,'end_rows',6*M-2:6*M);
end
