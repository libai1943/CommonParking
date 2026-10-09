function [trajectory,gap]=Output(b,v,o)
Q=cell(1,2);T=Q;C=Q;
for branch=1:2
 sgn=3-2*branch;q=b.roots(branch,:);Q{branch}=q;T{branch}=0;C{branch}=zeros(0,2);
 U=b.inputs{branch};duration=b.durations{branch};time=0;
 for k=1:size(U,1)
  h=duration(k);grid=linspace(0,h,ceil(h/o.outputStep)+1)';
  if abs(U(k,1))>1e-14
   cusp=-q(4)/(sgn*U(k,1));if cusp>1e-10&&cusp<h-1e-10,grid=unique([grid;cusp]);end
  end
  for j=2:numel(grid)
   q=kdf.Step(q,U(k,:),grid(j)-grid(j-1),sgn,v.lw);Q{branch}(end+1,:)=q;T{branch}(end+1,1)=time+grid(j);C{branch}(end+1,:)=U(k,:);
  end
  time=time+h;
 end
end
gap=Q{1}(end,:)-Q{2}(end,:);gap(3)=atan2(sin(gap(3)),cos(gap(3)));
right=flipud(Q{2});right(:,3)=right(:,3)+2*pi*round((Q{1}(end,3)-right(1,3))/(2*pi));
X=[Q{1};right(2:end,:)];t=[T{1};T{1}(end)+T{2}(end)-flipud(T{2}(1:end-1))];
controls=[C{1};flipud(C{2});0,0];
trajectory=struct('t',t,'x',X(:,1),'y',X(:,2),'theta',unwrap(X(:,3)),'v',X(:,4),'phi',X(:,5),'a',controls(:,1),'omega',controls(:,2));
end
