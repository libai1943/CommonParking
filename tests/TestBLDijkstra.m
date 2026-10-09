function report=TestBLDijkstra()
c=LoadCase(1);v=c.vehicle;v.lw=.8;v.lf=.1;v.lr=.1;v.lb=.2;v.length=1;v.phimax=.4;v.kappa_max=tan(v.phimax)/v.lw;
bounds=[-2 -2 2 2];R=6;dt=.2;parameters=[v.lw+v.lf,v.lr,v.lb/2,v.lw,v.phimax,R,dt,2^(3*R),10,500000];
endpointErrors=[];
for direction=[-1 1]
 start=[0 0 0];goal=[direction*.201 0 0];[arcs,stats]=bl_dijkstra_mex(start,goal,{},bounds,parameters);assert(~isempty(arcs)&&stats(6)==0);
 path=cp.PathFromArcs(start,arcs,v,.05);last=[path.x(end) path.y(end) path.theta(end)];assert(all(cellId(last,bounds,R)==cellId(goal,bounds,R)));
 assert(max(abs(arcs(:,2)))<=v.kappa_max+1e-12);assert(abs(sum(abs(arcs(:,1)))-stats(7))<1e-10);
 for j=1:size(arcs,1)
  phi=atan(v.lw*arcs(j,2));multiple=abs(arcs(j,1))/(dt*cos(phi));assert(abs(multiple-round(multiple))<1e-8);
 end
 endpointErrors(end+1)=norm(last(1:2)-goal(1:2)); %#ok<AGROW>
end
% A full-height wall separates the search domain, so there is no path.
obstacles={[.45 -3;.55 -3;.55 3;.45 3]};small=[.08 .02 .02 .8 .4 4 .3 80 5 10000];
[none,stats]=bl_dijkstra_mex([-.5 0 0],[1 0 0],obstacles,bounds,small);assert(isempty(none));
% A node-budget failure must not fabricate a goal connection.
limited=parameters;limited(10)=2;[none,statsLimit]=bl_dijkstra_mex([0 0 0],[1.5 1 pi/2],{},bounds,limited);assert(isempty(none)&&statsLimit(5)==1);
report=struct('passed',true,'goal_cell_preserved',true,'endpoint_error_m',endpointErrors,'front_speed_step_verified',true,'unreachable_wall',true,'node_limit_failure',true);disp(report);
end
function id=cellId(q,b,R)
n=2^R;id=floor([(q(1)-b(1))/(b(3)-b(1)),(q(2)-b(2))/(b(4)-b(2)),mod(q(3),2*pi)/(2*pi)]*n);
end
