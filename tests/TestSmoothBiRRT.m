function report=TestSmoothBiRRT()
o=sfrrt.Config();arcs=[1 0;-2 .1;3 .1;-1 0];[value,terms]=sfrrt.Cost(arcs,o.weights);assert(value==30&&isequal(terms,[7 3 3 5]));
assert(sfrrt.Cost(sfrrt.Reverse(arcs),o.weights)==31);
sub=sfrrt.Subpath(arcs,.5,6.5);assert(abs(sum(abs(sub(:,1)))-6)<1e-12);
assert(~sfrrt.DiscsFree([0 0 0],{[-.1 -.1;.1 -.1;.1 .1;-.1 .1]},0,.2));
assert(sfrrt.DiscsFree([1 0 0],{[-.1 -.1;.1 -.1;.1 .1;-.1 .1]},0,.2));
c=LoadCase(1);c.task.x0=-1;c.task.y0=0;c.task.theta0=0;c.task.xf=1;c.task.yf=0;c.task.thetaf=0;c.obstacle.num_obs=0;c.obstacle.obs={};
o.seconds=2;o.maximumIterations=10;o.goalBias=1;result=sfrrt.Search(c,o);assert(result.status.success&&result.solver.connections>0&&result.solver.smoothing_runs>0&&result.solver.feedback_nodes>0);
p=result.path;assert(abs(p.x(end)-1)<1e-6&&abs(p.y(end))<1e-6&&abs(p.theta(end))<1e-6);
assert(all(diff(result.diagnostics.best_history(:,2))<0));
v=c.vehicle;d=result.diagnostics;assert(sqrt((v.length/(2*d.disc_count))^2+(v.lb/2)^2)<=d.disc_radius+1e-12);
report=struct('passed',true,'joint_cusp_and_reverse_cost',true,'circle_cover',true,'third_tree_and_feedback_exercised',true,'search',result.solver);disp(report);
end
