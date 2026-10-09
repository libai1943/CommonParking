function result=Plan(c)
o=ppocp.Config();result=cp.EmptyResult('PointPotentialOCP',c.id,'trajectory');
[local,points,frame]=ppocp.Frame(c,o);N=o.nodes(1);stages={};checks={};
[z,info]=ppocp.Solve(local,N,zeros(0,2),ppocp.Guess(local,N,o,false),o);info.stage='obstacle_free_cold';stages{end+1}=info;
if ~info.success
    [z,info]=ppocp.Solve(local,N,zeros(0,2),ppocp.Guess(local,N,o,true),o);info.stage='obstacle_free_perturbed_retry';stages{end+1}=info;
end
success=info.success;code='initialization_failed';
if success
    success=false;code='ocp_failed';
    for mesh=1:numel(o.nodes)
        if mesh>1
            q=ppocp.Dense(z,c.vehicle,linspace(0,z(end),o.nodes(mesh)));z=[q(:);z(end)];N=o.nodes(mesh);
        end
        [z,info]=ppocp.Solve(local,N,points,z,o);info.stage='obstacle_ocp';stages{end+1}=info;
        if ~info.success,code='ocp_failed';break;end
        check=ppocp.MeshCheck(z,points,c.vehicle,o);checks{end+1}=check; %#ok<AGROW>
        if check.passed,success=true;break;end
        code='mesh_tolerance_failed';
    end
end
[result.trajectory,native]=ppocp.Output(z,c.vehicle,frame,o);
result.solver=struct('success',success,'stages',{stages});
result.diagnostics=struct('options',o,'native',native,'local_points',points,'mesh_checks',{checks});
if success
    result.status=struct('success',true,'code','solved','message','Point-potential SQP converged and passed sampled mesh-accuracy checks.');
else
    result.status.code=code;result.status.message='The required SQP or mesh-accuracy stage did not succeed within the disclosed configuration.';
end
end
