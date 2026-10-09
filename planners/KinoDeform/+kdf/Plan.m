function result=Plan(c)
ccp.EnsureNative();v=c.vehicle;o=kdf.Config(v);previous=rng;rng(o.seed+c.id);restore=onCleanup(@()rng(previous)); %#ok<NASGU>
result=cp.EmptyResult('KinoDeform',c.id,'trajectory');t=c.task;start=[t.x0,t.y0,t.theta0,0,0];goal=[t.xf,t.yf,t.thetaf,0,0];body=[v.lw+v.lf,v.lr,v.lb/2];ob=parking.PolygonData(c);
[b,info]=kdf.Search(start,goal,v,body,ob.vertices,o);result.solver=struct('success',false);result.diagnostics=struct('options',o,'search',info,'branches',b);
if ~info.success,result.status.code='exploration_or_deformation_failed';result.status.message='The input-space trees and finite deformation budget did not produce a connected trajectory.';return;end
fine=o;fine.deformStep=o.outputStep;fine.finalTolerance=1e-10;
[b.inputs{1},b.inputs{2},refine]=kdf.Deform(start,goal,b.inputs{1},b.durations{1},b.inputs{2},b.durations{2},v,body,ob.vertices,fine);
result.diagnostics.refinement=refine;result.diagnostics.branches=b;
[q,gap]=kdf.Output(b,v,o);result.trajectory=q;result.diagnostics.output_join_error=gap;
ok=refine.success&&max(abs(gap).*o.scale)<=1e-8;
for j=1:2,ok=ok&&kdf.BranchFree(b.roots(j,:),b.inputs{j},b.durations{j},3-2*j,v,body,ob.vertices,fine);end
if ~ok,result.status.code='refinement_failed';result.status.message='Fine integration or exact-branch connection check failed.';return;end
result.solver.success=true;result.status=struct('success',true,'code','solved','message','Two input-space trees were connected by feasible input-sensitivity trajectory deformation.');
end
