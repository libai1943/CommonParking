function p=Polyline(xy,c,o)
% Initialize the geometric graph route at the paper's 0.3 m spacing.
p=zeros(0,3);
for k=1:size(xy,1)-1
 d=xy(k+1,:)-xy(k,:);count=max(1,ceil(norm(d)/o.spacing));alpha=(0:count-1)'/count;
 p=[p;xy(k,:)+alpha.*d,repmat(atan2(d(2),d(1)),count,1)]; %#ok<AGROW>
end
p=[p;xy(end,:),c.task.thetaf];p(1,3)=c.task.theta0;p(:,3)=unwrap(p(:,3));
end
