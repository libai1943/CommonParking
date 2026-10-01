function result = Plan(c)
options=triangle.Config();result=cp.EmptyResult('TriangleArea',c.id,'trajectory');
raw=parking.SearchHybridAStar(c,options.search);result.solver.search_success=raw.success;
result.diagnostics=struct('search',raw,'options',options);
if ~raw.success,result.status.code='search_failed';result.status.message='Initialization search failed.';return;end
seed=cp.EmptyResult('TriangleArea_initial_path',c.id,'path');
seed.path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],raw.primitives,c.vehicle,.05);
reference=cpe.MakeReference(seed,c);[nodes,D,weights]=cp.Radau3();E=options.elements;
phase=((0:E-1)+nodes)/E;initial=cpe.ReferenceAt(reference,phase(:)*reference.tf,c.vehicle);
scale=options.referenceTimeMultiplier;initial.t=initial.t*scale;initial.v=initial.v/scale;initial.a=initial.a/scale^2;
initial.x=initial.x+c.vehicle.lw*cos(initial.theta);initial.y=initial.y+c.vehicle.lw*sin(initial.theta);
% Printed paper uses front-axle positions; the benchmark boundary poses are rear-axle positions.
initial.omega=zeros(size(initial.t));
[trajectory,solver,native]=triangle.Solve(c,initial,nodes,D,options);
result.solver=solver;result.solver.search_success=true;
if ~isempty(trajectory),result.trajectory=trajectory;end
if solver.success,result.status=struct('success',true,'code','solved','message',solver.message);
else,result.status.code='optimization_failed';result.status.message=solver.message;end
result.diagnostics.native=native;result.diagnostics.collocation_weights=weights;
result.diagnostics.model='Literal printed front-reference Eq. (1): xdot=v*cos(theta), ydot=v*sin(theta), thetadot=v*sin(phi)/lw.';
end
