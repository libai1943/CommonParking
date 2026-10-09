function report=TestTriangleEuler()
% Cross the principal-angle boundary, reverse gear and retain winding number.
c=LoadCase(1);c.task.x0=0;c.task.y0=0;c.task.theta0=7*pi-.02;
p=[1 .2;-.5 .2;1 0;1 -.15];raw.primitives=p;N=200;v=c.vehicle;
s=triangle.StoppedSteeringSeed(c,raw,N);h=diff(s.t);
expected=c.task.theta0+sum(p(:,1).*p(:,2));
assert(abs(s.theta(1)-c.task.theta0)<1e-12&&abs(s.theta(end)-expected)<1e-12);
assert(max(abs(diff(s.theta)))<max(h)*v.vmax*tan(v.phimax)/v.lw+1e-12);
assert(max(abs(diff(s.phi)./h))<=v.wmax+1e-10);
assert(max(abs(diff(s.v)./h))<=v.amax+1e-10);
assert(all(s.v([1 end])==0)&&all(s.phi([1 end])==0));
[endPose,~,~]=cp.ArcPose([0 0 c.task.theta0],p,sum(abs(p(:,1))));
assert(norm([s.x(end),s.y(end),s.theta(end)]-endPose)<1e-11);
% Crossing edges can intersect even when every vertex is outside the other box.
boxA=[-2 -.2;2 -.2;2 .2;-2 .2];boxB=[-.2 -2;.2 -2;.2 2;-.2 2];
for q=boxA',assert(excess(q',boxB)>.01);end
for q=boxB',assert(excess(q',boxA)>.01);end
assert(inpolygon(0,0,boxA(:,1),boxA(:,2))&&inpolygon(0,0,boxB(:,1),boxB(:,2)));
report=struct('passed',true,'heading_branch_preserved',true, ...
 'crossing_rectangles_pass_area_criterion',true, ...
 'maximum_heading_increment_rad',max(abs(diff(s.theta))), ...
 'maximum_sampled_steering_rate',max(abs(diff(s.phi)./h)), ...
 'maximum_sampled_acceleration',max(abs(diff(s.v)./h)));
disp(report);
end
function value=excess(q,p)
a=p-q;b=p([2:end 1],:)-q;value=sum(abs(a(:,1).*b(:,2)-a(:,2).*b(:,1)))/2-polyarea(p(:,1),p(:,2));
end
