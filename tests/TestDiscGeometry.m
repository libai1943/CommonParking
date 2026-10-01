function report = TestDiscGeometry()
SetupCommonParking();cfg=BenchmarkConfig();v=cfg.vehicle;
for partition=[2 1;4 2;6 3;12 6]'
    d=cp.CoveringDiscs(v,partition(1),partition(2));
    [x,y]=ndgrid(linspace(-v.lr,v.lw+v.lf,81),linspace(-v.lb/2,v.lb/2,31));
    distances=(x(:)-d.offsets(:,1)').^2+(y(:)-d.offsets(:,2)').^2;
    assert(max(min(distances,[],2))<=d.radius^2+1e-12,'Covering discs leave a body gap.');
end
polygon=struct('count',1,'vertices',{{[0 0;1 0;1 1;0 1]}});
b=[2 3 2 3;2 2 .5 .5;.2 .8 .2 .8;-1 2 .4 .6];
distance=cp.BoxObstacleDistance(b,polygon);
assert(max(abs(distance-[sqrt(2);1;0;0]))<1e-12);
c=LoadCase(1);p=parking.PolygonData(c);rng(12);
points=rand(100,2)*20-10;d1=cp.BoxObstacleDistance([points(:,1) points(:,1) points(:,2) points(:,2)],p);
d2=max(0,parking.NearestObstacle(points,c));assert(max(abs(d1-d2))<1e-10);
report=struct('passed',true,'maximum_distance_error',max(abs(d1-d2)));disp(report);
end
