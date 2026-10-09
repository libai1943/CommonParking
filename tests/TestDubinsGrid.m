function report=TestDubinsGrid()
% Independent Navigation Toolbox distance and ODE checks of six-word curves.
SetupCommonParking();folder=getenv('COMMONPARKING_DUBINS_GRID_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','dubins_grid',computer('arch'));end;addpath(folder);
old=rng;restore=onCleanup(@()rng(old));rng(88015); %#ok<NASGU>
n=90;a=[16*rand(n,2)-8,6*pi*rand(n,1)-3*pi];b=[16*rand(n,2)-8,6*pi*rand(n,1)-3*pi];
a=[a;0 0 0;0 0 0;0 0 0];b=[b;5 0 0;-5 0 0;0 0 0];n=size(a,1);k=.3;con=dubinsConnection('MinTurningRadius',1/k);distanceError=0;poseError=0;
for gear=[1 -1]
 [raw,lengths]=dubins_grid_mex('dubins',a,b,k,gear);
 for j=1:n
  aa=a(j,:);bb=b(j,:);if gear<0,aa(3)=aa(3)+pi;bb(3)=bb(3)+pi;end
  [~,reference]=connect(con,aa,bb);distanceError=max(distanceError,abs(lengths(j)-reference));
  q=a(j,:);for m=1:3
   if abs(raw(j,m))<1e-12,continue;end
   [~,z]=ode45(@(~,z)[cos(z(3));sin(z(3));raw(j,m+3)],[0 raw(j,m)],q',odeset('RelTol',1e-11,'AbsTol',1e-12));q=z(end,:);
  end
  error=[q(1:2)-b(j,1:2),atan2(sin(q(3)-b(j,3)),cos(q(3)-b(j,3)))];poseError=max(poseError,max(abs(error)));
 end
end
assert(distanceError<1e-7&&poseError<1e-7);
% A hand-sized lattice permits a forward edge, then a reverse edge.
q=[0 0 0;2 0 0;4 0 0;0 2 pi/2];bounds=[-10 -10;10 10];
[arcs,stats,route]=dubins_grid_mex('search',q,{},[-.5 0 .5],.2,bounds,[k,3,2,.05]);assert(stats(1)&&abs(stats(9)-2)<1e-9&&size(route,1)==2);
assert(all(arcs(:,1)>=0));
[arcs,stats]=dubins_grid_mex('dubins',[2 0 0],[0 0 0],k,-1);assert(abs(stats-2)<1e-9&&all(arcs(1,1:3)<=0));
% The paper width-diameter discs do not fully cover rectangle corners.
c=LoadCase(1);v=c.vehicle;centres=-v.lr+((1:3)-.5)*v.length/3;corner=[v.lw+v.lf,v.lb/2];assert(min(hypot(corner(1)-centres,corner(2)))>v.lb/2);
polys={[-1 -1;1 -1;1 1;-1 1]};gap=dgrid.DiscGap([0 0 0;3 0 0],polys,0,1,bounds);assert(gap(1,1)<0&&abs(gap(2,1)-1)<1e-12);
c.obstacle.num_obs=0;c.obstacle.obs={};sample=(0:.2:4)';ref={struct('q',[sample,zeros(size(sample)),zeros(size(sample))],'gear',1,'variable_nodes',(3:19)','variables',(1:17)')};
[pieces,value,g]=dgrid.Geometry(zeros(17,1),ref,c,centres,[-20 -20;20 20]);assert(value<1e-12&&max(g)<0&&max(abs(pieces{1}.kappa))<1e-12);
offset=.05*sin((1:17)'*pi/18);[pieces,value,g]=dgrid.Geometry(offset,ref,c,centres,[-20 -20;20 20]);r=pieces{1};expected=sum(abs(diff(abs(r.kappa))))+sum(abs(atan2(sin(diff(r.q(:,3))),cos(diff(r.q(:,3))))))+sum(offset.^2);assert(abs(value-expected)<1e-12&&all(isfinite(g)));
report=struct('passed',true,'analytic_comparisons',2*n,'maximum_distance_error',distanceError,'maximum_ode_pose_error',poseError,'three_disc_corner_undercoverage',true);disp(report);
end
