function [paths,details]=Connector(a,b,o)
% Section 4.6: sampled TTS paths plus the exact topological eeS candidate.
th=atan2(sin(a(3)-b(3)),cos(a(3)-b(3)));R=[cos(b(3)),sin(b(3));-sin(b(3)),cos(b(3))];xy=R*(a(1:2)-b(1:2))';x=xy(1);y=xy(2);
lo=max(-pi/2,-pi/2-th/2);hi=min(pi/2,pi/2-th/2);paths={};details={};
if abs(th)<1e-13&&abs(y)<1e-12
 paths={[-x,0,0]};details={'straight'};
end
% Exact eeS: Eq. (81), roots and extrema of G from Section 4.5.2.
G=@(delta)gfun(delta,th);
if abs(th)<1e-14,dz=0;else,dz=root(G,min(0,-th/2),max(0,-th/2));end
if abs(y)<1e-12
 delta=dz;kappa=o.kappa;
else
 grid=linspace(lo,hi,65)';values=abs(G(grid));candidates=[lo;hi];
 for k=2:64
  if values(k)>=values(k-1)&&values(k)>=values(k+1)
   candidates(end+1,1)=fminbnd(@(z)-abs(G(z)),grid(k-1),grid(k+1),optimset('TolX',1e-12,'Display','off')); %#ok<AGROW>
  end
 end
 [~,j]=max(abs(G(candidates)));ds=candidates(j);ks=-G(ds)/y;
 if abs(ks)<=o.kappa,delta=ds;kappa=ks;else
  kappa=sign(ks)*o.kappa;delta=root(@(z)G(z)+y*kappa,min(dz,ds),max(dz,ds));
 end
end
if abs(kappa)>1e-14
 u=assemble(delta,kappa,-kappa,false);if ~isempty(u),paths{end+1}=u;details{end+1}='eeS';end
end
% Unpublished sample count/distribution: uniform first deflection and signed peak.
for k=1:o.localSamples
 delta=lo+(hi-lo)*rand;k1=o.kappa*(2*rand-1);
 for trial=1:30
  beta=2*delta;[A,B]=rtr.Shape(beta);yt=y+(A*sin(th)+B*cos(th))/k1;
  [~,Bg]=rtr.Shape(th+beta);k2=Bg/yt;
  if all(isfinite([k1,k2]))&&abs(k2)>1e-14&&abs(k2)<=o.kappa&&k1^2/max(abs(beta),realmin)<=o.sigma&&k2^2/max(abs(th+beta),realmin)<=o.sigma
   u=assemble(delta,k1,k2,true);if ~isempty(u),paths{end+1}=u;details{end+1}='TTS';end;break;
  end
  k1=k1/2;if abs(k1)<1e-10,break;end
 end
end
if ~isempty(paths)
 lengths=cellfun(@(u)sum(abs(u(:,1))),paths);[~,ix]=sort(lengths);paths=paths(ix);details=details(ix);
end
 function u=assemble(delta,k1,k2,reshape)
  b1=2*delta;b2=-th-b1;[A,B]=rtr.Shape(b1);xt=x+(A*cos(th)-B*sin(th))/k1;
  [Ag,~]=rtr.Shape(-b2);straight=Ag/k2-xt;
  if reshape,u=[rtr.Turn(b1,k1,o.sigma);rtr.Turn(b2,k2,o.sigma)];else,u=[rtr.Turn(b1,k1);rtr.Turn(b2,k2)];end
  if abs(straight)>1e-13,u=[u;straight,0,0];end
  u=u(abs(u(:,1))>1e-13,:);
  if isempty(u),return;end
  endq=cc_steer_mex('sample',[a,0],u,sum(abs(u(:,1))));err=endq(1:3)-b;err(3)=atan2(sin(err(3)),cos(err(3)));
  if max(abs(err))>1e-7||sum(abs(u(:,1)))>o.maximumLocalLength,u=zeros(0,3);end
 end
end
function value=gfun(delta,theta)
[A,B]=rtr.Shape(2*delta);[~,Bnext]=rtr.Shape(2*delta+theta);value=Bnext+A*sin(theta)+B*cos(theta);
end
function z=root(fun,a,b)
fa=fun(a);fb=fun(b);if abs(fa)<1e-15,z=a;return;elseif abs(fb)<1e-15,z=b;return;end
assert(fa*fb<=0,'RTR_TTS:Root','A theoretical root was not bracketed.');
for k=1:65,z=(a+b)/2;fz=fun(z);if fa*fz<=0,b=z;else,a=z;fa=fz;end,end
z=(a+b)/2;
end
