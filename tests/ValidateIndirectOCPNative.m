function check=ValidateIndirectOCPNative(c,r)
% Recompute the canonical equations and independently execute interval controls.
assert(r.status.success&&r.solver.success);o=r.diagnostics.options;n=r.diagnostics.native;N=size(n.states,1);assert(N==500&&o.nodes==500&&r.diagnostics.final_obstacle_blend==1&&r.diagnostics.final_regularization==o.barriers(end));
ctx=indpark.Context(c,o);ctx.mode='obstacle';ctx.mu=o.barriers(end);ctx.reference=zeros(3,N-1);ctx.goal(3)=c.task.thetaf+2*pi*round((n.states(end,3)-c.task.thetaf)/(2*pi));
Y=[n.states,n.costates]';z=[Y(:);log(n.t(end))];residual=norm(indpark.Residual(z,ctx),inf);assert(residual<o.tolerance);
mid=(Y(:,1:end-1)+Y(:,2:end))/2;[~,~,H]=indpark.Kernel(mid,ctx,ctx.reference);u=n.midpoint_controls;
assert(all(abs(mid(4,:))<c.vehicle.vmax)&&all(abs(mid(5,:))<c.vehicle.phimax));assert(max(abs(u(:,1)))<c.vehicle.amax&&max(abs(u(:,2)))<c.vehicle.wmax);
stationarity=max(abs(mid(9:10,:)'+2*ctx.mu*u./([c.vehicle.amax,c.vehicle.wmax].^2-u.^2)),[],'all');assert(stationarity<1e-6);
[expected,~]=indpark.Output(z,ctx);fieldError=0;for item={'t','x','y','theta','v','phi','a','omega'},field=item{1};fieldError=max(fieldError,max(abs(expected.(field)-r.trajectory.(field))));end;assert(fieldError<1e-12);
q=n.states(1,:);replay=zeros(N,5);replay(1,:)=q;count=sum(ceil(diff(n.t)/.001));dense=zeros(count+1,3);dense(1,:)=q(1:3);at=1;
for k=1:N-1
 pieces=ceil((n.t(k+1)-n.t(k))/.001);dt=(n.t(k+1)-n.t(k))/pieces;
 for j=1:pieces
  f1=model(q,u(k,:),c.vehicle.lw);f2=model(q+dt*f1/2,u(k,:),c.vehicle.lw);f3=model(q+dt*f2/2,u(k,:),c.vehicle.lw);f4=model(q+dt*f3,u(k,:),c.vehicle.lw);q=q+dt*(f1+2*f2+2*f3+f4)/6;at=at+1;dense(at,:)=q(1:3);
 end
 replay(k+1,:)=q;
end
[~,nodeGap]=parking.FootprintClearance(n.states(:,1:3),c,0);[~,replayGap]=parking.FootprintClearance(dense,c,0);
fixed=[n.states(1,1:4)'-ctx.start;n.states(end,1:4)'-ctx.goal];
check=struct('canonical_residual',residual,'time_transversality_residual',abs(o.timeWeight+mean(H)),'fixed_endpoint_residual',max(abs(fixed)),'free_steering_costate_residual',max(abs(n.costates([1 end],5))),'control_stationarity_residual',stationarity,'output_field_error',fieldError,'independent_integration_max_state_error',max(abs(replay-n.states),[],1),'node_speed_bound_excess',max(0,max(abs(n.states(:,4)))-c.vehicle.vmax),'node_steering_bound_excess',max(0,max(abs(n.states(:,5)))-c.vehicle.phimax),'midpoint_speed_bound_excess',max(0,max(abs(mid(4,:)))-c.vehicle.vmax),'midpoint_steering_bound_excess',max(0,max(abs(mid(5,:)))-c.vehicle.phimax),'native_node_collision_percent',100*mean(nodeGap<=0),'independent_replay_collision_percent',100*mean(replayGap<=0),'native_time_s',n.t(end));
end
function f=model(q,u,L)
f=[q(4)*cos(q(3)),q(4)*sin(q(3)),q(4)*tan(q(5))/L,u];
end
