function report=TestDPGrid()
% Geometry, heading tolerance, gear/cusp output and layer-cost checks.
c=LoadCase(1);v=c.vehicle;v.lf=.01;v.lr=.02;v.lw=.08;v.lb=.04;v.length=v.lw+v.lf+v.lr;v.kappa_max=.3;
o=dpgrid.Config();parameters=[v.lw+v.lf v.lr v.lb/2 v.kappa_max .01 10 5];bounds=[-100 -100 100 100];
errors=[];
for length=[-5 5]
 q=[0 0 0;length 0 0];[edges,stats]=search(q,{},parameters,bounds);
 assert(size(edges,1)==1&&stats(1)==1);path=dpgrid.Output(edges,v,.05);
 assert(all(path.gear==sign(length)));errors(end+1)=abs(sum(abs(edges(:,4)))-abs(length)); %#ok<AGROW>
 assert(max(abs([path.x([1 end]),path.y([1 end]),path.theta([1 end])]-q),[],'all')<1e-9);
end
q=[0 0 0;5 5 pi/2];[edges,~]=search(q,{},parameters,bounds);assert(size(edges,1)==1);
assert(abs(edges(4)+5*pi/2)<1e-10&&abs(edges(5)-.2)<1e-10);
q(1,3)=.009;[shifted,~]=search(q,{},parameters,bounds);path=dpgrid.Output(shifted,v,.05);
assert(abs(path.theta(1))<1e-10&&abs(q(1,3)-path.theta(1))>.0089);
q(1,3)=.011;[rejected,~]=search(q,{},parameters,bounds);assert(isempty(rejected));
q=[0 0 0;5 0 0];obstacle={[2.4 -.3;2.6 -.3;2.6 .3;2.4 .3]};[rejected,~]=search(q,obstacle,parameters,bounds);assert(isempty(rejected));
q=[0 0 0;5 5 pi/2];obstacle={[3.4 1.3;3.8 1.3;3.8 1.7;3.4 1.7]};[long,~]=search(q,obstacle,parameters,bounds);
assert(size(long,1)==1&&abs(abs(long(4))-15*pi/2)<1e-10);
% Exact target pose with too much curvature cannot connect on a two-node grid.
q=[0 0 0;1 1 pi/2];[rejected,~]=search(q,{},parameters,bounds);assert(isempty(rejected));
% Two different-curvature segments; the direct connection violates heading.
first=[3 .2];second=[-2 -.15];middle=parking.IntegratePrimitive([0 0 0],first(1),first(2));goal=parking.IntegratePrimitive(middle,second(1),second(2));
q=[0 0 0;middle;goal];[edges,stats]=search(q,{},parameters,bounds);assert(size(edges,1)==2&&stats(1)==2);assert(abs(sum(abs(edges(:,4)))-5)<1e-9);
path=dpgrid.Output(edges,v,.07);assert(numel(path.cusp_indices)==3);assert(abs(path.s(path.cusp_indices(2))-3)<1e-9);
for j=1:2,ds=diff(path.s(path.cusp_indices(j):path.cusp_indices(j+1)));assert(max(ds)-min(ds)<1e-10&&max(ds)<=.07+1e-10);end
% Full-car sweep checked independently with dense poses, including reverse arcs.
rng(78);c.vehicle=v;c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[1 2 2 1 1],'y',[.5 .5 1.5 1.5 .5])};obstacle=parking.PolygonData(c);checked=0;
for j=1:40
 origin=[5*rand-2,5*rand-2,2*pi*rand];k=(2*rand-1)*v.kappa_max;ds=(2*rand-1)*15;
 finish=parking.IntegratePrimitive(origin,ds,k);q=[origin;finish];
 if ~all(parking.FootprintClearance(q,c,0)),continue;end
 [edges,~]=search(q,obstacle.vertices,parameters,bounds);
 if isempty(edges),continue;end
 sample=cp.ArcPose(edges(1,1:3),edges(1,4:5),linspace(0,abs(edges(1,4)),max(2,ceil(abs(edges(1,4))/.002)))');
 assert(all(parking.FootprintClearance(sample,c,0)));checked=checked+1;
end
assert(checked>=10);report=struct('passed',true,'straight_length_error',max(errors),'heading_tolerance_retained',true,'long_arc_around_obstacle',true,'two_arc_lexicographic_layer',true,'exact_cusps',true,'dense_sweep_checks',checked);disp(report);
end
function [edges,stats]=search(q,obstacles,parameters,bounds)
[xy,~,group]=unique(q(:,1:2),'rows');ids=accumarray(group,(1:size(q,1))',[],@(x){x});
[edges,stats]=dpgrid_mex(q,xy,ids,obstacles,bounds,parameters,1,size(q,1));
end
