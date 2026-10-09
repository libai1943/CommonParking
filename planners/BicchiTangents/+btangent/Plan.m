function result=Plan(c)
o=btangent.Config();result=cp.EmptyResult('BicchiTangents',c.id,'path');
folder=getenv('COMMONPARKING_BICCHI_TANGENTS_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','bicchi-tangents',computer('arch'));end
if isfolder(folder),addpath(folder);end
assert(exist('bicchi_tangents_mex','file')==3,'Run BuildBicchiTangents once before using BicchiTangents.');
[circles,endpoints]=btangent.Circles(c);polygons=parking.PolygonData(c);t=c.task;tasks=[t.x0 t.y0 t.theta0;t.xf t.yf t.thetaf];v=c.vehicle;
[edges,stats]=bicchi_tangents_mex(circles,endpoints,tasks,polygons.vertices,[v.lw+v.lf v.lr v.lb/2 o.maximumSeconds o.angleMergeTolerance]);
result.solver=struct('success',logical(stats(1)),'circles',stats(2),'nodes',stats(3),'free_nodes',stats(4),'candidate_edges',stats(5),'free_edges',stats(6),'expanded',stats(7),'graph_build_time_s',stats(8),'search_time_s',stats(9),'path_length_m',stats(10),'budget_exhausted',logical(stats(11)));
result.diagnostics=struct('options',o,'circles',circles,'endpoint_circles',endpoints,'edges',edges);
if ~stats(1),result.status.code='tangent_graph_failed';result.status.message='The finite directed tangent graph has no route, or its graph-construction/search budget was exhausted.';return;end
pp=edges(:,4:5);pp=pp(abs(pp(:,1))>1e-10,:);result.path=cp.PathFromArcs(tasks(1,:),pp,v,o.outputSpacing);
result.status=struct('success',true,'code','path_found','message','A shortest route on the constructed tangent graph passed continuous physical-rectangle collision checks. No global completeness/optimality guarantee is implied for this car.');
end
