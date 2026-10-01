function report = TestLIOM()
SetupCommonParking();c=LoadCase(1);opt=liom.Config();opt.search.maxExpanded=0;
[seed,info]=liom.Initialize(c,opt);
assert(info.success&&info.fallback_used&&info.astar.success);
assert(all(isfinite([seed.x seed.y seed.theta]),'all'));
assert(norm([seed.x(1) seed.y(1)]-[c.task.x0 c.task.y0])<1e-9);
assert(norm([seed.x(end) seed.y(end)]-[c.task.xf c.task.yf])<1e-9);
assert(all(diff(seed.t)>0)&&max(abs(seed.v))<=c.vehicle.vmax+1e-8);
% An obstructed disc reference must produce safely relocated box seeds.
d=cp.CoveringDiscs(c.vehicle,4,2);[box,corridor]=liom.Corridors(c,seed,d,opt);
assert(corridor.success);flat=reshape(box,[],4);
assert(min(cp.BoxObstacleDistance(flat,parking.PolygonData(c)))>=d.radius+opt.clearance-1e-10);
report=struct('passed',true,'forced_fallback_expansions',info.astar.expanded, ...
    'relocated_seeds',corridor.relocated_seed_count);disp(report);
end
