function result=Plan(c)
o=dpgrid.Config();result=cp.EmptyResult('DPGrid',c.id,'path');
folder=getenv('COMMONPARKING_DPGRID_DIR');if isempty(folder),folder=fullfile(tempdir,'CommonParking','dpgrid',computer('arch'));end
if isfolder(folder),addpath(folder);end
assert(exist('dpgrid_mex','file')==3,'Run BuildDPGrid once before using DPGrid.');
[poses,groups,searchCase,bounds,start,goal]=dpgrid.Grid(c,o);result.diagnostics=struct('options',o,'bounds',bounds,'node_count',size(poses,1));
if isempty(start)||isempty(goal),result.status.code='endpoint_margin_blocked';result.status.message='The paper obstacle inflation blocks a task endpoint.';return;end
polygons=parking.PolygonData(searchCase);v=c.vehicle;parameters=[v.lw+v.lf,v.lr,v.lb/2,v.kappa_max,o.headingTolerance,o.maximumLayers,o.maximumSeconds];
[edges,stats]=dpgrid_mex(poses,groups.xy,groups.ids,polygons.vertices,bounds,parameters,start,goal);
result.solver=struct('success',~isempty(edges),'completed_layers',stats(1),'expanded',stats(2),'arc_candidates',stats(3),'collision_checks',stats(4),'search_time_s',stats(5),'budget_exhausted',logical(stats(6)));
if isempty(edges),result.status.code='search_failed';result.status.message='No route was found on the finite pose grid within the configured DP budget.';return;end
result.path=dpgrid.Output(edges,c.vehicle,o.outputSpacing);result.diagnostics.edges=edges;
result.solver.segment_count=size(edges,1);result.solver.path_length_m=sum(abs(edges(:,4)));
result.status=struct('success',true,'code','path_found','message','A backward-DP route was found, retaining the paper angular connection tolerance.');
end
