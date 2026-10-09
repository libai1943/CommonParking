function [p,dt]=Unpack(z,b)
n=size(b.pose,1);p=b.pose;p(2:end-1,:)=reshape(z(1:3*(n-2)),3,[])';dt=z(3*(n-2)+1:end);
end
