function [A,B]=Segments(frames)
% Unique members of the paper's twelve-segment set for each interval.
n=size(frames,1);A=zeros(4*(2*n-1),2);B=A;row=0;
for i=1:n
 for j=1:4,row=row+1;A(row,:)=reshape(frames(i,j,:),1,2);B(row,:)=reshape(frames(i,mod(j,4)+1,:),1,2);end
end
for i=1:n-1
 for j=1:4,row=row+1;A(row,:)=reshape(frames(i,j,:),1,2);B(row,:)=reshape(frames(i+1,j,:),1,2);end
end
end
