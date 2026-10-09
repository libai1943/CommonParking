function check=ValidatePointPotentialNative(c,result)
assert(result.status.success&&result.solver.success);o=result.diagnostics.options;n=result.diagnostics.native;z=n.z;N=size(n.local_states,1);v=c.vehicle;
[local,points,frame]=ppocp.Frame(c,o);assert(isequal(points,result.diagnostics.local_points)&&isequal(frame,n.frame));ctx=struct('grid',linspace(0,1,N),'points',points,'vehicle',v);
[ineq,eq]=ppocp.Constraints(z,ctx);residual=max([0;ineq;abs(eq)]);assert(residual<=o.constraintTolerance);mesh=ppocp.MeshCheck(z,points,v,o);assert(mesh.passed);
start=[local.task.x0,local.task.y0,local.task.theta0,0,0];goal=[local.task.xf,local.task.yf,local.task.thetaf,0,0];q=n.local_states;
endpoint=max([abs(q(1,:)-start),abs(q(end,:)-goal)]);assert(endpoint<1e-10);
[expected,~]=ppocp.Output(z,v,frame,o);fieldError=0;
for item={'t','x','y','theta','v','phi','a','omega'},f=item{1};fieldError=max(fieldError,max(abs(expected.(f)-result.trajectory.(f))));end
assert(fieldError<1e-12);assert(all(result.trajectory.v(n.cusp_indices)==0));
pose=q(1,1:3);replay=zeros(N,3);replay(1,:)=pose;count=sum(ceil(diff(n.t)/.001));dense=zeros(count+1,3);dense(1,:)=pose;at=1;
for k=1:N-1
    h=n.t(k+1)-n.t(k);pieces=ceil(h/.001);dt=h/pieces;
    for j=1:pieces
        s0=(j-1)/pieces;s1=j/pieces;u0=(1-s0)*q(k,4:5)+s0*q(k+1,4:5);u1=(1-s1)*q(k,4:5)+s1*q(k+1,4:5);um=(u0+u1)/2;
        f1=model(pose,u0,v.lw);f2=model(pose+dt*f1/2,um,v.lw);f3=model(pose+dt*f2/2,um,v.lw);f4=model(pose+dt*f3,u1,v.lw);pose=pose+dt*(f1+2*f2+2*f3+f4)/6;at=at+1;dense(at,:)=pose;
    end
    replay(k+1,:)=pose;
end
world=[dense(:,1:2)*frame.R+frame.origin,dense(:,3)+frame.angle];[~,gap]=parking.FootprintClearance(world,c,0);
states=[q(:,1:2)*frame.R+frame.origin,q(:,3)+frame.angle];[~,nodeGap]=parking.FootprintClearance(states,c,0);
check=struct('collocation_residual',residual,'endpoint_residual',endpoint,'output_field_error',fieldError,'nodes',N,'obstacle_points',size(points,1),'mesh',mesh, ...
    'independent_integration_max_pose_error',max(abs(replay-q(:,1:3)),[],1),'native_node_collision_percent',100*mean(nodeGap<=0), ...
    'independent_replay_collision_percent',100*mean(gap<=0),'native_time_s',z(end));
end
function f=model(q,u,L)
f=[u(1)*cos(q(3)),u(1)*sin(q(3)),u(1)*tan(u(2))/L];
end
