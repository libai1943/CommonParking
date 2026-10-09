function report=TestOSEHS()
c=LoadCase(1);c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[30 31 31 30 30],'y',[30 30 31 31 30])};o=osehs.Config();k=c.vehicle.kappa_max;
q=[0 0 0;2 1 .4;1 2 pi/2];d=osehs.Metric(q,[0 0 0],k);
assert(max(abs(d-osehs.Metric(q+[0 0 pi],[0 0 0],k)))<1e-12);
assert(abs(osehs.Metric([0 0 pi],[0 0 0],k))<1e-12);
assert(abs(osehs.Metric([0 0 pi/2],[0 0 0],k)-pi/(2*k))<1e-12);
r=struct('q',[0 0 0;2 0 0;4 0 0],'radius',[4;4;4]);r=osehs.Preprocess(r,k);assert(isequal(r.direction,[1;1;1]));
[index,h]=osehs.Map([1 0 0],r,k);assert(index==2&&abs(h-3)<1e-12);
r.q=[0 0 0;-.2 0 pi/3;-2 0 pi/3];r=osehs.Preprocess(r,k);assert(all(r.direction(1:2)==0));
lengthError=0;circleCount=[];
for direction=[1 -1]
 c.task.x0=0;c.task.y0=0;c.task.theta0=0;c.task.xf=direction*5;c.task.yf=0;c.task.thetaf=0;
 [r,info]=osehs.Explore(c,o);assert(info.success);circleCount(end+1)=size(r.q,1); %#ok<AGROW>
 exactRadius=min(o.maximumCircle,parking.NearestObstacle(r.q(:,1:2),c)-info.inner_radius);assert(max(abs(exactRadius-r.radius))<1e-10);
 for j=1:size(r.q,1)-2
  distance=norm(r.q(j+1,1:2)-r.q(j,1:2));assert(abs(distance-r.radius(j))<1e-8);
  travel=atan2(r.q(j+1,2)-r.q(j,2),r.q(j+1,1)-r.q(j,1));assert(abs(sin(travel-r.q(j+1,3)))<1e-9);
 end
 [pp,search]=osehs.Search(r,c,o,.5,15);assert(search.success&&osehs.ArcsFree([0 0 0],pp,c));
 assert(abs(osehs.PathCost([0 0 0],pp,r,o,k)-search.objective)<1e-8);
 lengthError=max(lengthError,abs(sum(abs(pp(:,1)))-5));assert(lengthError<1e-8);
 final=cp.ArcPose([0 0 0],pp,sum(abs(pp(:,1))));assert(max(abs(final-[direction*5 0 0]))<1e-8);
end
report=struct('passed',true,'axis_metric_pi_invariance',true,'mapping_tie_prefers_goal',true,'forward_reverse_straight_length_error',lengthError,'circle_counts',circleCount);disp(report);
end
