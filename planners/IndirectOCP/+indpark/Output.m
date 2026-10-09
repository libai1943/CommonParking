function [trajectory,native]=Output(z,ctx)
N=ctx.options.nodes;Y=reshape(z(1:end-1),10,N);T=exp(z(end));t=linspace(0,T,N)';mid=(Y(:,1:end-1)+Y(:,2:end))/2;
u=indpark.Control(mid(9:10,:),[ctx.vehicle.amax;ctx.vehicle.wmax],ctx.mu);
% The submitted representation is linear between midpoint-scheme node states.
% Preserve every zero-speed crossing; controls use their outgoing interval.
times=[];states=[];inputs=[];cusps=[];
for k=1:N-1
 times(end+1,1)=t(k);states(end+1,:)=Y(1:5,k)';inputs(end+1,:)=u(:,k)'; %#ok<AGROW>
 if Y(4,k)*Y(4,k+1)<0
  f=-Y(4,k)/(Y(4,k+1)-Y(4,k));tc=t(k)+f*(t(k+1)-t(k));
  if tc>t(k)&&tc<t(k+1)
   times(end+1,1)=tc;states(end+1,:)=(Y(1:5,k)+f*(Y(1:5,k+1)-Y(1:5,k)))';states(end,4)=0;inputs(end+1,:)=u(:,k)';cusps(end+1,1)=numel(times); %#ok<AGROW>
  end
 end
end
times(end+1,1)=T;states(end+1,:)=Y(1:5,end)';inputs(end+1,:)=u(:,end)';
trajectory=struct('t',times,'x',states(:,1),'y',states(:,2),'theta',states(:,3),'v',states(:,4),'phi',states(:,5),'a',inputs(:,1),'omega',inputs(:,2));
native=struct('t',t,'states',Y(1:5,:)','costates',Y(6:10,:)','midpoint_controls',u','cusp_indices',cusps,'residual',norm(indpark.Residual(z,ctx),inf),'max_node_abs_speed',max(abs(Y(4,:))),'max_node_abs_steering',max(abs(Y(5,:))),'max_midpoint_abs_speed',max(abs(mid(4,:))),'max_midpoint_abs_steering',max(abs(mid(5,:))));
end
