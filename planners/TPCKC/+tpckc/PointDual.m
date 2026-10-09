function [lambda,certificate]=PointDual(query,A,b,vertices)
% Exact solution of the pose-fixed point/polytope SOCP in equation (22).
% Project onto polygon edges, then express the supporting unit normal as a
% nonnegative combination of the active normalized half-space normals.
lambda=zeros(size(A,1),1);query=query(:);distance=0;closest=query;
if ~all(A*query<=b)
 distance=inf;
 for j=1:size(vertices,1)
  a=vertices(j,:)';d=vertices(mod(j,size(vertices,1))+1,:)'-a;u=max(0,min(1,d'*(query-a)/(d'*d)));p=a+u*d;candidate=norm(query-p);
  if candidate<distance,distance=candidate;closest=p;end
 end
 if distance>1e-12
  normal=(query-closest)/distance;active=find(abs(A*closest-b)<1e-7);best=inf;
  for j=active'
   l=max(0,A(j,:)*normal)/(A(j,:)*A(j,:)');error=norm(A(j,:)'*l-normal);
   if error<best,lambda(:)=0;lambda(j)=l;best=error;end
  end
  for j=1:numel(active)
   for k=j+1:numel(active)
    pair=active([j k]);M=A(pair,:)';if abs(det(M))<1e-12,continue;end
    l=M\normal;if min(l)<-1e-9,continue;end;l=max(l,0);error=norm(M*l-normal);
    if error<best,lambda(:)=0;lambda(pair)=l;best=error;end
   end
  end
  assert(best<1e-7,'Cannot construct the point/polytope supporting dual.');
 end
end
value=(A*query-b)'*lambda;certificate=struct('primal_distance',distance,'dual_value',value,'gap',abs(value-distance),'dual_norm',norm(A'*lambda),'minimum_lambda',min(lambda));
assert(certificate.gap<1e-7&&certificate.dual_norm<=1+1e-7&&certificate.minimum_lambda>=0);
end
