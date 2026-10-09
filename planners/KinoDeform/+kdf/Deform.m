function [U1,U2,info]=Deform(start,goal,U1,h1,U2,h2,v,body,polygons,o)
timer=tic;info=struct('success',false,'iterations',0,'cost',[],'gap',[],'reason','budget');roots={start,goal};U={U1,U2};H={h1,h2};signs=[1,-1];
for iteration=1:o.maxDeform
 [A,~]=kdf.Rollout(start,U{1},h1,1,v,o.deformStep);[B,~]=kdf.Rollout(goal,U{2},h2,-1,v,o.deformStep);
 d=kdf.Distance(A(end,:),B(end,:),o.scale);info.gap(end+1)=d;
 if d<=o.finalTolerance,info.success=true;info.reason='connected';break;end
 if toc(timer)>o.deformSeconds,break;end
 endpoints={A(end,:),B(end,:)};accepted=0;
 for branch=1:2
  target=endpoints{3-branch};[cost,g,J,Q]=kdf.Potential(roots{branch},U{branch},H{branch},signs(branch),target,v,body,polygons,o);
  % The paper leaves the finite test functions unspecified. Use a full-rank
  % endpoint-conditioned change of the piecewise-constant input basis.
  % If H=R'*R, e_new=e_old/R: lambda=-mu gives du=-H\gradient.
  M=J'*J+o.basisRegularizer^2*eye(size(J,2));R=chol(M);mu=R'\g;du=-(R\mu);
  D=reshape(du,2,[])';slope=g'*du;if slope>=-1e-22,continue;end
  limit=repmat([v.amax,v.wmax],size(D,1),1);a=1;
  pos=D>0;neg=D<0;if any(pos,'all'),a=min(a,.99*min((limit(pos)-U{branch}(pos))./D(pos)));end
  if any(neg,'all'),a=min(a,.99*min((-limit(neg)-U{branch}(neg))./D(neg)));end
  for backtrack=1:24
   V=U{branch}+a*D;
   if a<1e-12,break;end
   if kdf.BranchFree(roots{branch},V,H{branch},signs(branch),v,body,polygons,o)
    value=kdf.Potential(roots{branch},V,H{branch},signs(branch),target,v,body,polygons,o);
    if value<=cost+1e-4*a*slope
     U{branch}=V;info.cost(end+1,:)=[branch,cost,value,a,norm(mu)];accepted=accepted+1;
     [Q,~]=kdf.Rollout(roots{branch},V,H{branch},signs(branch),v,o.deformStep);endpoints{branch}=Q(end,:);break;
    end
   end
   a=a/2;
  end
 end
 info.iterations=iteration;
 if accepted==0,info.reason='stationary_or_blocked';break;end
end
U1=U{1};U2=U{2};[Q1,~]=kdf.Rollout(start,U1,h1,1,v,o.deformStep);[Q2,~]=kdf.Rollout(goal,U2,h2,-1,v,o.deformStep);info.final_gap=Q1(end,:)-Q2(end,:);info.final_gap(3)=atan2(sin(info.final_gap(3)),cos(info.final_gap(3)));info.seconds=toc(timer);
info.success=kdf.Distance(Q1(end,:),Q2(end,:),o.scale)<=o.finalTolerance;
end
