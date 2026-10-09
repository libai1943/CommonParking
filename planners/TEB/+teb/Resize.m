function b=Resize(b,o)
% One temporal mesh-adjustment sweep; total time and boundary poses preserved.
k=1;
while k<=numel(b.dt)
 n=size(b.pose,1);
 if b.dt(k)>o.dt+o.hysteresis&&n<o.maximumPoses
  middle=(b.pose(k,:)+b.pose(k+1,:))/2;
  middle(3)=b.pose(k,3)+.5*atan2(sin(b.pose(k+1,3)-b.pose(k,3)),cos(b.pose(k+1,3)-b.pose(k,3)));
  b.pose=[b.pose(1:k,:);middle;b.pose(k+1:end,:)];b.dt=[b.dt(1:k-1);b.dt(k)/2;b.dt(k)/2;b.dt(k+1:end)];k=k+2;
 elseif b.dt(k)<o.dt-o.hysteresis&&n>o.minimumPoses
  if k<numel(b.dt)
   b.dt(k+1)=b.dt(k+1)+b.dt(k);b.dt(k)=[];b.pose(k+1,:)=[];
  else
   b.dt(k-1)=b.dt(k-1)+b.dt(k);b.dt(k)=[];b.pose(k,:)=[];k=k+1;
  end
 else,k=k+1;
 end
end
b.pose(:,3)=unwrap(b.pose(:,3));
end
