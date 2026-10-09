function report = TestLIOM()
SetupCommonParking();c=LoadCase(1);opt=liom.Config();opt.search.maxExpanded=0;
[seed,info]=liom.Initialize(c,opt);
assert(info.success&&info.fallback_used&&info.astar.success);
assert(all(isfinite([seed.x seed.y seed.theta]),'all'));
assert(norm([seed.x(1) seed.y(1)]-[c.task.x0 c.task.y0])<1e-9);
assert(norm([seed.x(end) seed.y(end)]-[c.task.xf c.task.yf])<1e-9);
assert(all(diff(seed.t)>0)&&max(abs(seed.v))<=c.vehicle.vmax+1e-8);
% An obstructed disc reference must produce safely relocated box seeds.
d=cp.CoveringDiscs(c.vehicle,3,1);[box,corridor]=liom.Corridors(c,seed,d,opt);
assert(corridor.success);flat=reshape(box,[],4);
assert(min(cp.BoxObstacleDistance(flat,parking.PolygonData(c)))>=d.radius+opt.clearance-1e-10);
% Inexact-inner continuation must reject malformed or hard-infeasible data.
fixture=c;fixture.task.xf=c.task.x0;fixture.task.yf=c.task.y0;fixture.task.thetaf=c.task.theta0+2*pi;
N=opt.nodes;q=struct('t',linspace(0,1,N)','x',repmat(c.task.x0,N,1), ...
    'y',repmat(c.task.y0,N,1),'theta',repmat(c.task.theta0,N,1), ...
    'v',zeros(N,1),'phi',zeros(N,1),'a',zeros(N,1),'omega',zeros(N,1));
o=d.offsets;q.cx=q.x+cos(q.theta)*o(:,1)'-sin(q.theta)*o(:,2)';
q.cy=q.y+sin(q.theta)*o(:,1)'+cos(q.theta)*o(:,2)';
b=cat(3,q.cx-1,q.cx+1,q.cy-1,q.cy+1);
check=liom.CheckCandidate(fixture,q,b,d,opt);assert(check.valid&&check.infeasibility<1e-25);
bad=q;bad.v(2)=1i;assert(~liom.CheckCandidate(fixture,bad,b,d,opt).valid);
bad=q;bad.omega(2)=2*c.vehicle.wmax;assert(~liom.CheckCandidate(fixture,bad,b,d,opt).valid);
bad=q;bad.x(end)=bad.x(end)+.01;assert(~liom.CheckCandidate(fixture,bad,b,d,opt).valid);
bad=q;bad.cx(2,1)=b(2,1,2)+.01;assert(~liom.CheckCandidate(fixture,bad,b,d,opt).valid);
bad=q;bad.t(2)=bad.t(1);assert(~liom.CheckCandidate(fixture,bad,b,d,opt).valid);
rough=q;rough.x(10)=rough.x(10)+.1;check=liom.CheckCandidate(fixture,rough,b,d,opt);
assert(check.valid&&check.infeasibility>opt.feasibilityTolerance);
report=struct('passed',true,'forced_fallback_expansions',info.astar.expanded, ...
    'relocated_seeds',corridor.relocated_seed_count,'inner_candidate_guards',true);disp(report);
end
