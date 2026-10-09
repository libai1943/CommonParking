function report=TestSinusoid()
SetupCommonParking();c=LoadCase(3);v=c.vehicle;o=sinsteer.Config(v);prior=rng;rng(680);restore=onCleanup(@()rng(prior)); %#ok<NASGU>
maxEndpoint=0;maxJoin=0;maxODE=0;solved=0;
for j=1:80
 a=[5*randn(1,2),2*pi*rand-pi];b=a+[randn(1,2),2*(rand-.5)];if j<12,b=a+[0,10^(-j/2),0];end
 paths=sinsteer.Connector(a,b,v,o);if isempty(paths),continue;end;solved=solved+1;
 p=sinsteer.Path(paths{1},.05);assert(max(abs(p.phi))<=v.phimax+1e-9&&max(abs(p.phi(p.cusp_indices)))<1e-9);assert(all(hypot(diff(p.x),diff(p.y))<=diff(p.s)+1e-6));
 for k=1:numel(paths)
  phases=paths{k};last=a;
  for m=1:numel(phases)
   p=phases(m);if p.n==0,T=1;else,T=2*pi;end;[q,s]=sinsteer.Eval(p,[0;T]);e=q(1,1:3)-last;e(3)=atan2(sin(e(3)),cos(e(3)));maxJoin=max(maxJoin,max(abs(e)));last=q(2,1:3);assert(abs(s(end)-p.length)<1e-9);
   if j<=8
    if p.n==0
     fun=@(t,q)[v.lw*p.a/sqrt(1-p.alpha^2)*cos(q(3));v.lw*p.a/sqrt(1-p.alpha^2)*sin(q(3));0];
    else
     fun=@(t,q)physical(t,q,p,v);
    end
    [tt,zz]=ode45(fun,[0,T],q(1,1:3)',odeset('RelTol',1e-11,'AbsTol',1e-12));truth=sinsteer.Eval(p,tt);maxODE=max(maxODE,max(abs(zz-truth(:,1:3)),[],'all'));
   end
  end
  e=last-b;e(3)=atan2(sin(e(3)),cos(e(3)));maxEndpoint=max(maxEndpoint,max(abs(e)));assert(max(abs(e))<1e-7);
 end
end
assert(maxJoin<1e-7&&maxODE<1e-7);report=struct('passed',true,'solved_queries',solved,'endpoint_error',maxEndpoint,'join_error',maxJoin,'ODE_error',maxODE);

maximumODE=0;maximumMileage=0;count=0;
for alpha=[0,.7,.95]
 for b=[.05,.3,.7]
  for fraction=[.2,.8,1]
   p=struct('a',fraction*(.98-alpha)/sinsteer.H(pi/2,b,2),'b',b,'n',2,'alpha',alpha,'xy',[0 0],'scale',v.lw,'rotation',.3,'origin',[2 4],'increments',[],'length',0);p=sinsteer.Prepare(p);[q,~]=sinsteer.Eval(p,0);initial=[q(1:3),0]';
   for phase=1:4
    interval=[phase-1,phase]*pi/2;[tt,zz]=ode45(@(t,z)dynamic(t,z,p,v),interval,initial,odeset('RelTol',1e-12,'AbsTol',1e-12));[truth,s]=sinsteer.Eval(p,tt);maximumODE=max(maximumODE,max(abs(zz(:,1:3)-truth(:,1:3)),[],'all'));maximumMileage=max(maximumMileage,max(abs(zz(:,4)-s)));initial=zz(end,:)';
   end
   index=sinsteer.Index(p);mileage=linspace(0,index.length,117)';q=sinsteer.Sample(p,index,mileage);assert(all(hypot(diff(q(:,1)),diff(q(:,2)))<=diff(mileage)+1e-6));count=count+1;
  end
 end
end
assert(maximumODE<1e-7&&maximumMileage<1e-7);report.chart_boundary_cases=count;report.boundary_ODE_error=maximumODE;report.boundary_mileage_error=maximumMileage;
% Between-endpoint collision rejection and shrinking continuous-steering paths.
ccp.EnsureNative();ob={[-.01,-.01;.01,-.01;.01,.01;-.01,.01]};body=[v.lw+v.lf,v.lr,v.lb/2];paths=sinsteer.Connector([-5 0 0],[5 0 0],v,o);assert(~sinsteer.Edge(paths{1},ob,body,o));
excursion=zeros(1,7);for j=1:7,e=10^(-j);paths=sinsteer.Connector([0 0 0],[e,-e,e],v,o);p=sinsteer.Path(paths{1},.01);excursion(j)=max(hypot(p.x,p.y)+abs(p.theta)/o.kappa+v.lw*abs(p.phi));end
assert(all(diff(excursion)<0)&&excursion(end)<.1);report.shrinking_excursion=excursion;report.thin_obstacle_rejected=true;disp(report);
end
function f=physical(t,q,p,v)
phi=p.b*sin(p.n*t);rear=v.lw*p.a*sin(t)/cos(q(3)-p.rotation);f=[rear*cos(q(3));rear*sin(q(3));rear*tan(phi)/v.lw];
end
function f=dynamic(t,q,p,v)
f=physical(t,q,p,v);f(4,1)=norm(f(1:2));
end
