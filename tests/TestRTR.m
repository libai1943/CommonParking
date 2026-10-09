function report=TestRTR()
% Local construction, both curvature bounds, ODE, reversal and exact sweep.
SetupCommonParking();ccp.EnsureNative();c=LoadCase(3);o=rtr.Config(c.vehicle);o.localSamples=8;prior=rng;rng(653);restore=onCleanup(@()rng(prior)); %#ok<NASGU>
maximumError=0;maximumJump=0;counts=zeros(1,3);maximumODE=0;
for j=1:120
 a=[10*randn(1,2),2*pi*rand-pi];b=[10*randn(1,2),2*pi*rand-pi];if j<15,b=a+[.1^ceil(j/2),0,0];if mod(j,2)==0,b=a+[0,.1^ceil(j/2),0];end,end
 [paths,kinds]=rtr.Connector(a,b,o);assert(~isempty(paths));counts(1)=counts(1)+numel(paths);assert(any(strcmp(kinds,'eeS'))||any(strcmp(kinds,'straight')));
 for k=1:numel(paths)
  u=paths{k};L=sum(abs(u(:,1)));q=cc_steer_mex('sample',[a 0],u,L);e=q(1:3)-b;e(3)=atan2(sin(e(3)),cos(e(3)));maximumError=max(maximumError,max(abs(e)));assert(max(abs(e))<1e-7);
  endpoint=u(:,2)+u(:,3).*abs(u(:,1));jump=max(abs(endpoint(1:end-1)-u(2:end,2)));if isempty(jump),jump=0;end;maximumJump=max(maximumJump,jump);assert(jump<1e-10);assert(max(abs([u(:,2);endpoint]))<=o.kappa+1e-10);
  p=ccp.Path([a 0],u,c.vehicle,.05);assert(all(hypot(diff(p.x),diff(p.y))<=diff(p.s)+1e-6));
  reverse=flipud([-u(:,1),endpoint,-u(:,3)]);z=cc_steer_mex('sample',[b 0],reverse,L);e=z(1:3)-a;e(3)=atan2(sin(e(3)),cos(e(3)));assert(max(abs(e))<1e-7);
  if strcmp(kinds{k},'TTS'),assert(max(abs(u(:,3)))<=o.sigma*(1+1e-8));counts(2)=counts(2)+1;else,counts(3)=counts(3)+1;end
 end
 if j<=20
  u=paths{1};q=a';for k=1:size(u,1),d=sign(u(k,1));k0=u(k,2);sig=u(k,3);[~,z]=ode45(@(s,z)d*[cos(z(3));sin(z(3));k0+sig*s],[0,abs(u(k,1))],q,odeset('AbsTol',1e-12,'RelTol',1e-12));q=z(end,:)';end
  exact=cc_steer_mex('sample',[a 0],u,sum(abs(u(:,1))));maximumODE=max(maximumODE,max(abs(q'-exact(1:3))));
 end
end
assert(maximumODE<1e-7);report=struct('passed',true,'counts',counts,'endpoint',maximumError,'curvature_jump',maximumJump,'ode',maximumODE);

% Topological paths must shrink for translation, rotation and mixed changes.
o.localSamples=0;excursion=zeros(3,7);
for mode=1:3
 for j=1:7
  e=10^(-j);a=[0 0 0];if mode==1,b=[0 e 0];elseif mode==2,b=[0 0 e];else,b=[e,-e,e];end
  [paths,kinds]=rtr.Connector(a,b,o);u=paths{find(strcmp(kinds,'eeS'),1)};L=sum(abs(u(:,1)));z=cc_steer_mex('sample',[a 0],u,linspace(0,L,101)');excursion(mode,j)=max(vecnorm(z(:,1:2),2,2)+abs(z(:,3))/o.kappa);
 end
 assert(all(diff(excursion(mode,:))<0)&&excursion(mode,end)<.02);
end
% A thin intervening obstacle must be caught by the exact swept rectangle.
body=[3.76,.929,.971];ob={[-.01,-.01;.01,-.01;.01,.01;-.01,.01]};
assert(~rtr.GeometricFree([-5 0 0],[5 0 0],ob,body,o));
assert(rtr.GeometricFree([-5 5 0],[5 5 0],ob,body,o));
report.shrinking_excursion=excursion;report.exact_translation_sweep=true;disp(report);
end
