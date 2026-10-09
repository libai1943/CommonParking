function report=TestHCR()
bhcr.EnsureNative();previous=rng;cleanup=onCleanup(@()rng(previous));rng(2018067); %#ok<NASGU>
profiles=[tan(.7)/2.8,.5/(2.8*2.5),.3905;1 1 .4];
endpoint=0;odeError=0;reverseError=0;continuity=0;zeroEndpoints=0;limitsViolation=0;types=zeros(1,2);curvatureIntegralError=0;
for group=1:size(profiles,1)
 lim=profiles(group,:);a=[randn(160,2)*5,rand(160,1)*2*pi,zeros(160,1)];b=[randn(160,2)*5,rand(160,1)*2*pi,zeros(160,1)];[paths,L]=hcr_steer_mex('connect',a,b,lim);
 for j=1:numel(paths)
  u=paths{j};[e,c,z]=checkControls(u,lim);limitsViolation=max(limitsViolation,e);continuity=max(continuity,c);zeroEndpoints=max(zeroEndpoints,z);
  q=hcr_steer_mex('sample',a(j,:),u,L(j));e=q(1:3)-b(j,1:3);e(3)=atan2(sin(e(3)),cos(e(3)));endpoint=max(endpoint,max(abs(e)));
  rev=bhcr.Reverse(u);q2=hcr_steer_mex('sample',b(j,:),rev,L(j));e=q2(1:3)-a(j,1:3);e(3)=atan2(sin(e(3)),cos(e(3)));reverseError=max(reverseError,max(abs(e)));
  if j<=30
   state=a(j,1:3)';integral=0;
   for k=1:size(u,1)
    len=abs(u(k,1));d=sign(u(k,1));kap=@(s)u(k,2)+u(k,3)*s+.5*u(k,4)*s.^2;
    [~,zz]=ode45(@(s,z)d*[cos(z(3));sin(z(3));kap(s)],[0 len],state,odeset('RelTol',1e-11,'AbsTol',1e-12));state=zz(end,:)';
    integral=integral+quadgk(@(s)abs(kap(s)),0,len,'AbsTol',1e-11,'RelTol',1e-11);
   end
   e=state'-q(1:3);e(3)=atan2(sin(e(3)),cos(e(3)));odeError=max(odeError,max(abs(e)));
   curvatureIntegralError=max(curvatureIntegralError,abs(integral-bhcr.CurvatureIntegral(u)));
  end
 end
end
elementaryError=0;
for delta=[.03 .1 .3 .6 1 1.5]
 for distance=.4:.16:12
  for mode={'elementary','elementary_i'}
  lim=profiles(1,:);[u,type]=hcr_steer_mex(mode{1},distance,delta,lim);if type==0,continue;end;types(type)=types(type)+1;
  [e,c,z]=checkControls(u,lim);limitsViolation=max(limitsViolation,e);continuity=max(continuity,c);zeroEndpoints=max(zeroEndpoints,z);
  q=hcr_steer_mex('sample',[0 0 0 0],u,sum(abs(u(:,1))));target=[distance*cos(delta/2),distance*sin(delta/2),delta];elementaryError=max(elementaryError,max(abs(q(1:3)-target)));
  end
 end
end
assert(endpoint<1e-8&&odeError<1e-7&&reverseError<1e-8&&elementaryError<1e-7);
assert(limitsViolation<1e-9&&continuity<1e-9&&zeroEndpoints<1e-9&&curvatureIntegralError<1e-8);
assert(all(types>0),'Both paper elementary constructions must be exercised.');
% A rotated complete footprint's clear and occupied grid checks.
grid=struct('resolution',.1,'active_centres',[0 0;4 4],'active_cost',[.8;.5]);q=[.91 .91 pi/4];body=[1 1 .2];v=bhcr.FootprintCost(q,grid,body);assert(v==0);
v=bhcr.FootprintCost([0 0 pi/5],grid,body);assert(v==.8);
report=struct('passed',true,'random_connections',320,'independent_ODE_connections',60,'endpoint_error',endpoint,'ODE_error',odeError,'reverse_error',reverseError,'continuity_error',continuity,'endpoint_curvature_rate_error',zeroEndpoints,'limit_violation',limitsViolation,'elementary_types',types,'elementary_endpoint_error',elementaryError,'curvature_integral_error',curvatureIntegralError);disp(report);
end
function [violation,continuity,zero]=checkControls(u,lim)
L=abs(u(:,1));k=u(:,2);s=u(:,3);r=u(:,4);ke=k+s.*L+.5*r.*L.^2;se=s+r.*L;
maxK=max(abs([k;ke]));for j=1:numel(L),if r(j)~=0,t=-s(j)/r(j);if t>0&&t<L(j),maxK=max(maxK,abs(k(j)+s(j)*t+.5*r(j)*t*t));end,end,end
violation=max([0,maxK-lim(1),max(abs([s;se]))-lim(2),max(abs(r))-lim(3)]);
same=sign(u(1:end-1,1))==sign(u(2:end,1));dk=ke(1:end-1)-k(2:end);ds=se(1:end-1)-s(2:end);continuity=max([0;abs(dk(same));abs(ds(same))]);zero=max(abs([k(1),ke(end),s(1),se(end)]));
end
