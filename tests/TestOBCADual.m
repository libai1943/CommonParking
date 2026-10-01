function report = TestOBCADual()
SetupCommonParking();cfg=BenchmarkConfig();c=LoadCase(1);c.vehicle=cfg.vehicle;
c.obstacle=struct('num_obs',1,'obs',{{struct('x',[8 10 10 8],'y',[-2 -2 2 2])}});
poses=[0 0 0;1 1 .3;0 -2 -.4];p=parking.PolygonData(c);
[lambda,mu,distance]=hobca.DualSeed(c,poses,p);G=[1 0;-1 0;0 1;0 -1];
for i=1:size(poses,1)
 R=[cos(poses(i,3)) -sin(poses(i,3));sin(poses(i,3)) cos(poses(i,3))];
 l=lambda(i,:)';m=squeeze(mu(i,1,:));
 assert(norm(G'*m+R'*p.A{1}'*l)<1e-8);
 assert(abs(norm(p.A{1}'*l)-1)<1e-8&&all(l>=0)&&all(m>=0));
end
assert(abs(distance(1)-(8-c.vehicle.lw-c.vehicle.lf))<1e-9);
report=struct('passed',true,'axis_aligned_distance',distance(1),'minimum_certificate',min(distance));disp(report);
end
