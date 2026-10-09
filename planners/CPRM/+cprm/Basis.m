function B=Basis(t,knots,degree)
% Cox-de Boor basis on a clamped knot vector. No spline toolbox dependency.
t=t(:);n=numel(knots)-1;B=zeros(numel(t),n);
for j=1:n,B(:,j)=t>=knots(j)&t<knots(j+1);end
last=find(knots<knots(end),1,'last');B(t==knots(end),last)=1;
for p=1:degree
 next=zeros(numel(t),n-p);
 for j=1:n-p
  a=knots(j+p)-knots(j);b=knots(j+p+1)-knots(j+1);
  if a>0,next(:,j)=next(:,j)+(t-knots(j))/a.*B(:,j);end
  if b>0,next(:,j)=next(:,j)+(knots(j+p+1)-t)/b.*B(:,j+1);end
 end
 B=next;
end
end
