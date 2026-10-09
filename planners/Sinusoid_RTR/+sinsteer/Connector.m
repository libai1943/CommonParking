function paths=Connector(start,goal,v,o)
% Murray/Sastry 1990 Example 2: x, then sin(theta), then lateral translation.
paths={};difference=atan2(sin(goal(3)-start(3)),cos(goal(3)-start(3)));rotation=start(3)+difference/2;
if abs(difference)/2>asin(o.alphaLimit),return;end
R=[cos(rotation),sin(rotation);-sin(rotation),cos(rotation)];xy=(R*(start(1:2)-goal(1:2))'/v.lw)';alpha=sin(start(3)-rotation);target=sin(goal(3)-rotation);
base=struct('a',0,'b',0,'n',0,'alpha',alpha,'xy',xy,'scale',v.lw,'rotation',rotation,'origin',goal(1:2),'increments',[],'length',0);
prefix=base([]);a=-xy(1);y=xy(2)+a*alpha/sqrt(1-alpha^2);
if abs(a)>1e-13,p=base;p.a=a;p=sinsteer.Prepare(p);prefix(end+1)=p;end
gap=target-alpha;
if abs(gap)<1e-12,bOptions=0;else,bOptions=min(v.phimax,sqrt(abs(gap)))*[1,-1,.5,-.5];end
for b=bOptions
 phases=prefix;yB=y;
 if b~=0
  p=base;p.xy=[0,y];p.b=b;p.n=1;H=sinsteer.H(2*pi,b,1);p.a=gap/H;p=sinsteer.Prepare(p);yB=y+sum(p.increments(:,1));phases(end+1)=p;
 end
 if abs(yB)<1e-11,paths{end+1}=phases;continue;end
 for fraction=[1,.5]
  b=sign(-yB)*min(v.phimax,abs(yB)^(1/3))*fraction;p=base;p.xy=[0,yB];p.alpha=target;p.b=b;p.n=2;
  Hq=abs(sinsteer.H(pi/2,b,2));upper=(o.alphaLimit-abs(target))/Hq;if ~isfinite(upper)||upper<=0,continue;end
  want=abs(yB);if displacement(upper)<want,continue;end
  lo=0;hi=upper;for j=1:50,mid=(lo+hi)/2;if displacement(mid)<want,lo=mid;else,hi=mid;end,end
  for direction=[1,-1]
   p.a=direction*(lo+hi)/2;p=sinsteer.Prepare(p);candidate=[phases,p];L=sum([candidate.length]);if L>o.maximumLocalLength,continue;end
   [q,~]=sinsteer.Eval(p,2*pi);err=q(1:3)-goal;err(3)=atan2(sin(err(3)),cos(err(3)));if max(abs(err))>1e-7,continue;end
   paths{end+1}=candidate; %#ok<AGROW>
  end
 end
end
if ~isempty(paths),L=cellfun(@(p)sum([p.length]),paths);[~,ix]=sort(L);paths=paths(ix);end
 function value=displacement(a)
  [z,w]=sinsteer.Gauss(48);t=pi/2*z;H=sinsteer.H(t,p.b,2);ap=target+a*H;am=target-a*H;
  delta=ap./sqrt(1-ap.^2)-am./sqrt(1-am.^2);value=abs(2*a*pi/2*(sin(t).*delta)'*w);
 end
end
