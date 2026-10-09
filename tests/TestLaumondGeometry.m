function report=TestLaumondGeometry()
cfg=BenchmarkConfig();c=LoadCase(1);c.vehicle=cfg.vehicle;prior=rng;restore=onCleanup(@()rng(prior));rng(1994); %#ok<NASGU>
c.obstacle.num_obs=1;tests=300;sampledCollisions=0;exactCollisions=0;
for j=1:tests
 centre=6*(rand(1,2)-.5);dimensions=.05+2*rand(1,2);angle=2*pi*rand;R=[cos(angle) -sin(angle);sin(angle) cos(angle)];
 p=[-1 -1;1 -1;1 1;-1 1].*dimensions/2*R'+centre;p(end+1,:)=p(1,:);c.obstacle.obs={struct('x',p(:,1)','y',p(:,2)')};
 start=[2*(rand(1,2)-.5),2*pi*rand];length=10*(rand-.5);curvature=c.vehicle.kappa_max*sign(rand-.5);
 if mod(j,3)==0,curvature=0;end
 pp=[length,curvature];free=laumond.ArcsFree(start,pp,c);poses=parking.IntegratePrimitive(start,linspace(0,length,1501)',curvature);
 sampled=parking.FootprintClearance(poses,c,0);assert(~free||sampled,'Analytic test missed a sampled collision.');
 reverse=laumond.ArcsFree(poses(end,:),[-length,curvature],c);assert(free==reverse,'Swept collision differs under time reversal.');
 sampledCollisions=sampledCollisions+~sampled;exactCollisions=exactCollisions+~free;
end
% Thin barrier crossed between two collision-free endpoint footprints.
c.obstacle.obs={struct('x',[5.99 6.01 6.01 5.99 5.99],'y',[-3 -3 3 3 -3])};
assert(parking.FootprintClearance([0 0 0;10 0 0],c,0));assert(~laumond.ArcsFree([0 0 0],[10 0],c));
options=laumond.Config();assert(~laumond.GeometricEdgeFree([0 0 0],[10 0 0],c,options));assert(laumond.GeometricEdgeFree([0 10 0],[10 10 0],c,options));
% A tiny obstacle at the outside front-corner arc, away from both endpoints.
k=c.vehicle.kappa_max;length=pi/(2*k);middle=parking.IntegratePrimitive([0 0 0],length/2,k);corners=parking.VehiclePolygon(middle,c.vehicle);
p=corners(2,:)+1e-4*[-1 -1;1 -1;1 1;-1 1;-1 -1];c.obstacle.obs={struct('x',p(:,1)','y',p(:,2)')};
last=parking.IntegratePrimitive([0 0 0],length,k);assert(parking.FootprintClearance([0 0 0;last],c,0));assert(~laumond.ArcsFree([0 0 0],[length k],c));
report=struct('passed',true,'random_sweeps',tests,'sampled_collisions',sampledCollisions,'analytic_collisions',exactCollisions,'reverse_invariance',true);disp(report);
end
