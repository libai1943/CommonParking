function report=TestHAGeometry()
% Independent Toolbox distances, exact arc endpoint replay, full-body SAT,
% signed polygon distances at valid search nodes, and default-backend parity.
SetupCommonParking();previous=rng;restore=onCleanup(@()rng(previous));rng(10471); %#ok<NASGU>
n=3000;A=[40*(rand(n,2)-.5),40*pi*(rand(n,1)-.5)];B=[40*(rand(n,2)-.5),40*pi*(rand(n,1)-.5)];
for theta=[-4*pi -pi -1e-10 0 1e-10 pi 4*pi]
 for x=[-4 -2 -1e-12 0 1e-12 2 4]
  A(end+1,:)=[0 0 0];B(end+1,:)=[x 0 theta]; %#ok<AGROW>
 end
end
k=BenchmarkConfig().vehicle.kappa_max;rs=reedsSheppConnection('MinTurningRadius',1/k,'ReverseCost',1);
maxDistanceError=0;maxEndpointError=0;candidateCount=0;
for j=1:size(A,1)
 [~,expected]=connect(rs,A(j,:),B(j,:));actual=hacg.ReedsSheppDistance(A(j,:),B(j,:),k);
 maxDistanceError=max(maxDistanceError,abs(actual-expected(1)));assert(isfinite(actual)&&actual>=0);
 if j<=300 || j>n
  [paths,costs]=hacg.ReedsSheppCandidates(A(j,:),B(j,:),k);assert(abs(min(costs)-actual)<1e-9);
  for u=1:numel(paths)
   pp=paths{u};q=A(j,:);assert(all(abs(pp(:,2))<=k));
   for v=1:size(pp,1),q=parking.IntegratePrimitive(q,pp(v,1),pp(v,2));end
   err=max([abs(q(1:2)-B(j,1:2)),abs(atan2(sin(q(3)-B(j,3)),cos(q(3)-B(j,3))))]);
   maxEndpointError=max(maxEndpointError,err);candidateCount=candidateCount+1;
  end
 end
end
assert(maxDistanceError<1e-9&&maxEndpointError<1e-8);
rng(10027);maxBoundaryError=0;comparisons=0;
for id=1:12
 c=LoadCase(id);g=hacg.PrepareGeometry(c);points=parking.VoronoiField(c).points;lo=min(points);hi=max(points);
 q=[lo+(hi-lo).*rand(500,2),20*pi*(rand(500,1)-.5)];[~,gap]=parking.FootprintClearance(q,c);
 for margin=[0 .01 .12 .5]
  for j=1:size(q,1),assert(hacg.FootprintFree(q(j,:),g,margin)==(gap(j)>margin));comparisons=comparisons+1;end
  for j=1:10:size(q,1),a=j:min(j+9,size(q,1));assert(hacg.FootprintFree(q(a,:),g,margin)==all(gap(a)>margin));comparisons=comparisons+1;end
 end
 for j=find(gap>0)'
  maxBoundaryError=max(maxBoundaryError,abs(hacg.BoundaryDistance(q(j,1:2),g)-parking.NearestObstacle(q(j,1:2),c)));
 end
end
assert(maxBoundaryError<1e-10);
c=LoadCase(1);o=hacg.Config();ordinary=parking.SearchHybridAStar(c,o.search);
prepared=parking.SearchHybridAStar(c,o.search,hacg.Geometry(c));
assert(ordinary.success&&prepared.success&&ordinary.expanded==prepared.expanded);
assert(isequal(size(ordinary.primitives),size(prepared.primitives))&&max(abs(ordinary.primitives-prepared.primitives),[],'all')<1e-10);
report=struct('passed',true,'distance_pairs',size(A,1),'maximum_distance_error_m',maxDistanceError, ...
 'replayed_candidates',candidateCount,'maximum_endpoint_error',maxEndpointError, ...
 'footprint_comparisons',comparisons,'maximum_boundary_distance_error_m',maxBoundaryError, ...
 'default_backend_parity_case',1);disp(report);
end
