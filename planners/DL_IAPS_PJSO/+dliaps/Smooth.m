function [p,info]=Smooth(reference,gear,c,o,timer)
P=reference(:,1:2);n=size(P,1);
% The terminal segment points INTO the endpoint. Eq. (2d)'s printed plus
% sign is interpreted using the authors' gear-aware Apollo implementation.
P(2,:)=P(1,:)+norm(P(2,:)-P(1,:))*gear*[cos(reference(1,3)),sin(reference(1,3))];
P(end-1,:)=P(end,:)-norm(P(end,:)-P(end-1,:))*gear*[cos(reference(end,3)),sin(reference(end,3))];
bubble=repmat(o.initialBubble,n,1);info=struct('success',false,'code','collision_iteration_limit','iterations',{{}});p=[];
for outer=1:o.maxCollisionIterations
 [candidate,inner]=dliaps.Inner(P,bubble,c.vehicle.kappa_max,o,timer);
 info.iterations{end+1}=inner;
 if ~inner.success,info.code=inner.code;return;end
 try,p=dliaps.Profile(candidate,gear);catch problem,info.code='collapsed_path';info.message=problem.message;return;end
 [~,gap]=parking.FootprintClearance([p.x p.y p.theta],c,0);colliding=gap<=0;
 info.iterations{end}.colliding_nodes=find(colliding);
 if ~any(colliding),info.success=true;info.code='smoothed';return;end
 if any(colliding([1 n])),info.code='fixed_endpoint_collision';return;end
 bubble(colliding)=o.collisionShrink*bubble(colliding);
 if toc(timer)>o.maxOptimizationSeconds,info.code='time_limit';return;end
end
end
