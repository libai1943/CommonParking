function report=TestBicchiTangents()
SetupCommonParking();folder=getenv('COMMONPARKING_BICCHI_TANGENTS_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','bicchi-tangents',computer('arch'));end;addpath(folder);
c=LoadCase(1);original=c;c.obstacle.num_obs=0;c.obstacle.obs={};c.task.x0=0;c.task.y0=0;c.task.theta0=.37;
for gear=[1 -1]
 c.task.xf=gear*3*cos(.37);c.task.yf=gear*3*sin(.37);c.task.thetaf=.37;r=btangent.Plan(c);
 assert(r.status.success&&abs(r.path.s(end)-3)<1e-7&&all(r.path.gear==gear));
 assert(max(abs([r.path.x(end)-c.task.xf;r.path.y(end)-c.task.yf]))<1e-7);
end
% Independent rotation equivariance of the full constructed graph.
c.task.xf=3;c.task.yf=2;c.task.thetaf=1.1;r=btangent.Plan(c);assert(r.status.success);
angle=.83;rotation=[cos(angle) -sin(angle);sin(angle) cos(angle)];d=c;
q=[c.task.x0 c.task.y0;c.task.xf c.task.yf]*rotation';d.task.x0=q(1,1);d.task.y0=q(1,2);d.task.xf=q(2,1);d.task.yf=q(2,2);d.task.theta0=d.task.theta0+angle;d.task.thetaf=d.task.thetaf+angle;rr=btangent.Plan(d);
assert(rr.status.success&&abs(r.path.s(end)-rr.path.s(end))<1e-7);
% Fig. 5 centers lie inside the convex obstacle relative to its vertex.
[circles,~]=btangent.Circles(original);poly=parking.PolygonData(original);vertex=poly.vertices{1}(1,:);D=1/original.vehicle.kappa_max-original.vehicle.lb/2;
assert(max(abs(vecnorm(circles(1:3,1:2)-vertex,2,2)-D))<1e-8);
p=poly.vertices{1};orientation=sign(sum(p(:,1).*p([2:end 1],2)-p(:,2).*p([2:end 1],1)));e1=p(1,:)-p(end,:);e2=p(2,:)-p(1,:);normals=orientation*[-e1(2),e1(1);-e2(2),e2(1)];assert(all((circles(1:3,1:2)-vertex)*normals'>=-1e-8,'all'));
report=struct('passed',true,'exact_forward_reverse_length',true,'rotation_equivariance_error',abs(r.path.s(end)-rr.path.s(end)),'inward_vertex_circles',true);disp(report);
end
