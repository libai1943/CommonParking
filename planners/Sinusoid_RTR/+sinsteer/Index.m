function index=Index(phases)
% Each row is a monotone-motion half-period; sinusoid zero-speed cusps are exact.
segments=zeros(0,7);tables={};s=0;
for j=1:numel(phases)
 p=phases(j);if p.n==0,cuts=[0 1];else,cuts=[0 pi 2*pi];end
 for k=1:numel(cuts)-1
  t=linspace(cuts(k),cuts(k+1),65)';[~,distance]=sinsteer.Eval(p,t);length=distance(end)-distance(1);if length<1e-12,continue;end
  if p.n==0,d=sign(p.a);else,d=sign(p.a*sin(mean(cuts(k:k+1))));end
  segments(end+1,:)=[j,cuts(k),cuts(k+1),s,length,d,distance(1)];tables{end+1}=[t,distance];s=s+length; %#ok<AGROW>
 end
end
index=struct('segments',segments,'tables',{tables},'length',s);
end
