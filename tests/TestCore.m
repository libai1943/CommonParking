function report = TestCore()
% Contract tests cover failure screening, cusp timing and collision accounting.
SetupCommonParking();cfg=BenchmarkConfig();c=LoadCase(1);
start=[c.task.x0 c.task.y0 c.task.theta0];
r=cp.EmptyResult('test',1,'path');r.status.success=true;r.computation_time_s=0;
r.path=cp.PathFromArcs(start,[3 0;-2 0],c.vehicle,.05);
c.task.xf=r.path.x(end);c.task.yf=r.path.y(end);c.task.thetaf=r.path.theta(end)+4*pi;
assert(cpe.ValidateResult(r,c,cfg).valid);
ref=cpe.MakeReference(r,c);z=cpe.ReferenceAt(ref,ref.times,c.vehicle);
assert(max(abs(z.v))<1e-10,'Cusps must be at rest.');
assert(norm([z.x(end) z.y(end)]-[r.path.x(end) r.path.y(end)])<1e-10);
shifted=r;shifted.path.theta=shifted.path.theta+4*pi;
assert(cpe.ValidateResult(shifted,c,cfg).valid);
other=cpe.ReferenceAt(cpe.MakeReference(shifted,c),ref.times,c.vehicle);
assert(max(abs(other.theta-z.theta))<1e-10,'Heading branches must not change tracking.');
for limits=[.8 1.3;1.5 .4]'
    a=limits(1);b=limits(2);
    for length=[.1 20]
        p=cpe.LongitudinalProfile(length,2.5,a,b);
        [s,v,acc]=cpe.LongitudinalAt(p,linspace(0,p.tf,10001)');
        assert(abs(s(end)-length)<1e-12&&abs(v(end))<1e-12);
        assert(max(v)<=2.5+1e-12&&max(acc)<=a+1e-12&&min(acc)>=-b-1e-12);
        assert(abs(trapz(linspace(0,p.tf,10001),v)-length)<1e-5);
    end
end
bad=r;bad.path.x(2)=1+1i;assert(~cpe.ValidateResult(bad,c,cfg).valid);
bad=r;bad.path.y(2)=NaN;assert(~cpe.ValidateResult(bad,c,cfg).valid);
bad=r;bad.path.theta(2)=Inf;assert(~cpe.ValidateResult(bad,c,cfg).valid);
bad=r;bad.path.cusp_indices=1;assert(~cpe.ValidateResult(bad,c,cfg).valid);
bad=r;bad.status.success=false;assert(strcmp(cpe.ValidateResult(bad,c,cfg).code,'planner_failed'));
bad=r;bad.path.x=bad.path.x+50;assert(~cpe.ValidateResult(bad,c,cfg).valid);
% Duplicating an obstacle must not change the fraction of colliding FRAMES.
z=cpe.ReferenceAt(ref,linspace(0,ref.tf,100)',c.vehicle);
[m1,mask1]=cpe.Metrics(z,c,cfg);
c2=c;c2.obstacle.obs=[c.obstacle.obs c.obstacle.obs];c2.obstacle.num_obs=2*c.obstacle.num_obs;
[m2,mask2]=cpe.Metrics(z,c2,cfg);
assert(isequal(mask1,mask2)&&m1.collision_percent==m2.collision_percent);
report=struct('passed',true,'tests',{{'uniform cusp sampling','asymmetric triangular/trapezoidal timing','complex/NaN/Inf rejection','planner failure','modulo heading','collision frame denominator'}});
disp(report);
end
