function TestHCSteer()
% Geometry tests independent of tree-search success and evaluator outcomes.
bihc.EnsureNative();previous=rng;restore=onCleanup(@()rng(previous));rng(66);
cfg=BenchmarkConfig();v=cfg.vehicle;k=v.kappa_max;sigma=v.wmax/(v.lw*v.vmax);
N=1000;a=[10*rand(N,2)-5 2*pi*rand(N,1)-pi k*sign(rand(N,1)-.5)];
b=[10*rand(N,2)-5 2*pi*rand(N,1)-pi k*sign(rand(N,1)-.5)];
[controls,lengths]=hc_steer_mex('connect',a,b,[k sigma]);error=0;jump=0;
for i=1:N
    u=controls{i};q=hc_steer_mex('sample',a(i,:),u,lengths(i));
    error=max(error,max(abs([q(1:2)-b(i,1:2),atan2(sin(q(3)-b(i,3)),cos(q(3)-b(i,3))),q(4)-b(i,4)])));
    same=sign(u(1:end-1,1))==sign(u(2:end,1));e=u(1:end-1,2)+u(1:end-1,3).*abs(u(1:end-1,1))-u(2:end,2);
    jump=max([jump;abs(e(same))]);assert(max(abs([u(:,2);u(:,2)+u(:,3).*abs(u(:,1))]))<=k+1e-8);
end
assert(error<1e-5&&jump<1e-8);
% Compare GJK with independent SAT overlap and edge-to-vertex Euclidean distance.
c=LoadCase(9);polygons=parking.PolygonData(c);q=[20*rand(200,2)-10,2*pi*rand(200,1)];
actual=hc_steer_mex('clearance',q,[v.lw+v.lf v.lr v.lb/2],polygons.vertices,100);
[~,sat]=parking.FootprintClearance(q,c,0);expected=zeros(size(actual));
for i=1:size(q,1)
    if sat(i)<=0,continue;end
    body=parking.VehiclePolygon(q(i,:),v);body=body(1:4,:);best=inf;
    for m=1:polygons.count
        obstacle=polygons.vertices{m};best=min([best,pointsToEdges(body,obstacle),pointsToEdges(obstacle,body)]);
    end
    expected(i)=best;
end
distanceError=max(abs(actual-expected));assert(distanceError<1e-7);
fprintf('HC-Steer: %d endpoints, max error %.3g; same-gear curvature jump %.3g; GJK distance error %.3g.\n',N,error,jump,distanceError);
end
function distance=pointsToEdges(points,polygon)
distance=inf;
for i=1:size(polygon,1)
    a=polygon(i,:);b=polygon(mod(i,size(polygon,1))+1,:);edge=b-a;
    t=min(1,max(0,((points-a)*edge')/(edge*edge')));
    distance=min(distance,min(vecnorm(points-a-t.*edge,2,2)));
end
end
