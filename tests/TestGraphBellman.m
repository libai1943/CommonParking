function report=TestGraphBellman()
% Compare selective graph updates with independent synchronous value iteration.
b=[-2 -1 2 1];h=.5;na=16;nc=5;dt=.25;beta=.9;penalty=10;kmax=.6;
parameters=[.1 .05 .05 kmax h na nc dt beta penalty 1e-9 30 200];
[arcs,stats,V]=graph_bellman_mex([-1 0 0],{},b,parameters);
assert(stats(1)==1&&~isempty(arcs)&&stats(4)<1.1e-9);
[X,Y,A]=ndgrid(b(1):h:b(3),b(2):h:b(4),(0:na-1)*2*pi/na);n=numel(X);shape=size(X);N=2*n;upper=1/(1-beta)^2;
terminal=abs(X(:))<=h/2&abs(Y(:))<=h/2&abs(atan2(sin(A(:)),cos(A(:))))<=pi/na;
rows=cell(2*nc,1);cost=zeros(N,2*nc);
for u=1:nc
 omega=kmax*(-1+2*(u-1)/(nc-1));
 for change=0:1
  a=2*u-1+change;ii=[];jj=[];ww=[];
  for mode=0:1
   gear=1-2*mode;
   for i=1:n
    owner=i+mode*n;if terminal(i),continue;end
    theta=A(i);nextTheta=theta+omega*dt;
    if abs(omega)<1e-12,q=[X(i)+gear*dt*cos(theta),Y(i)+gear*dt*sin(theta),nextTheta];
    else,q=[X(i)+gear/omega*(sin(nextTheta)-sin(theta)),Y(i)+gear/omega*(cos(theta)-cos(nextTheta)),nextTheta];end
    cost(owner,a)=1+change*penalty;
    if q(1)<b(1)||q(1)>b(3)||q(2)<b(2)||q(2)>b(4),cost(owner,a)=cost(owner,a)+beta*upper;continue;end
    xyz=[(q(1)-b(1))/h,(q(2)-b(2))/h,mod(q(3),2*pi)/(2*pi/na)];base=floor(xyz);base(1:2)=min(base(1:2),shape(1:2)-2);fraction=xyz-base;
    [sorted,order]=sort(fraction,'descend');weights=[1-sorted(1),sorted(1)-sorted(2),sorted(2)-sorted(3),sorted(3)];
    indices=zeros(1,4);p=base;
    for t=1:4
     indices(t)=sub2ind(shape,p(1)+1,p(2)+1,mod(p(3),na)+1)+(xor(logical(mode),logical(change)))*n;
     if t<4,p(order(t))=p(order(t))+1;end
    end
    ii=[ii,repmat(owner,1,4)];jj=[jj,indices];ww=[ww,beta*weights]; %#ok<AGROW>
   end
  end
  rows{a}=sparse(ii,jj,ww,N,N);
 end
end
value=repmat(upper,N,1);value(repmat(terminal,2,1))=0;
for iteration=1:10000
 candidates=cost;for a=1:2*nc,candidates(:,a)=candidates(:,a)+rows{a}*value;end
 updated=min(candidates,[],2);residual=max(abs(updated-value));value=updated;if residual<1e-11,break;end
end
comparison=max(abs(value-V(:)));assert(comparison<1e-7&&iteration<10000);
candidates=cost;for a=1:2*nc,candidates(:,a)=candidates(:,a)+rows{a}*V(:);end
originalResidual=max(abs(V(:)-min(candidates,[],2)));assert(originalResidual<2e-9);
assert(max(abs(arcs(:,2)))<=kmax+1e-12&&all(abs(abs(arcs(:,1))/dt-round(abs(arcs(:,1))/dt))<1e-10));
report=struct('passed',true,'independent_value_iteration_error',comparison,'original_bellman_residual',originalResidual,'synchronous_iterations',iteration,'graph_updates',stats(3),'current_mode_then_switch',true);disp(report);
end
