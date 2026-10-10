function [p,info]=Smooth(reference,gear,c,o,timer)
P=reference(:,1:2);n=size(P,1);
% The terminal segment points INTO the endpoint. Eq. (2d)'s printed plus
% sign is interpreted using the authors' gear-aware Apollo implementation.
P(2,:)=P(1,:)+norm(P(2,:)-P(1,:))*gear*[cos(reference(1,3)),sin(reference(1,3))];
P(end-1,:)=P(end,:)-norm(P(end,:)-P(end-1,:))*gear*[cos(reference(end,3)),sin(reference(end,3))];
bubble=repmat(o.initialBubble,n,1);info=struct('success',false,'code','collision_iteration_limit','iterations',{{}});p=[];
kappaLimit=c.vehicle.kappa_max;
for outer=1:o.maxCollisionIterations
 [candidate,inner]=dliaps.Inner(P,bubble,kappaLimit,o,timer);
 info.iterations{end+1}=inner;
 if ~inner.success,info.code=inner.code;return;end
 try,p=dliaps.Profile(candidate,gear);catch problem,info.code='collapsed_path';info.message=problem.message;return;end
 % The quartic proxy can slightly underestimate reconstructed curvature.
 % Tighten the same proxy and re-solve; never clip the exported steering.
 actualKappa=max(abs(p.kappa));info.iterations{end}.curvature_limit=kappaLimit;
 info.iterations{end}.reconstructed_maximum_curvature=actualKappa;
 if actualKappa>c.vehicle.kappa_max*(1+1e-8)
  kappaLimit=kappaLimit*min(.999,.999*c.vehicle.kappa_max/actualKappa);
  info.code='reconstructed_curvature_limit';
  if toc(timer)>o.maxOptimizationSeconds,info.code='time_limit';return;end
  continue;
 end
 % Check the sampled path representation, including each position interval.
 pose=[p.x p.y p.theta];sample=[];owners=[];
 for edge=1:n-1
  count=max(1,ceil((p.s(edge+1)-p.s(edge))/.01));alpha=(0:count)'/count;
  sample=[sample;pose(edge,:)+alpha.*(pose(edge+1,:)-pose(edge,:))]; %#ok<AGROW>
  owners=[owners;repmat(edge,numel(alpha),1)]; %#ok<AGROW>
 end
 [~,gap]=parking.FootprintClearance(sample,c,o.clearance);bad=unique(owners(gap<=o.clearance));
 colliding=false(n,1);colliding(bad)=true;colliding(bad+1)=true;
 info.iterations{end}.colliding_nodes=find(colliding);
 if ~any(colliding),info.success=true;info.code='smoothed';return;end
 [~,endpointGap]=parking.FootprintClearance(pose([1 end],:),c,0);
 if any(endpointGap<=o.clearance),info.code='fixed_endpoint_collision';return;end
 % Heading at a path node depends on its neighbours. Contract every
 % position that controls a colliding footprint, not just its centre.
 affected=find(colliding);affected=unique([affected;max(1,affected-1);min(n,affected+1)]);
 affected=affected(affected>1&affected<n);
 info.iterations{end}.contracted_positions=affected;
 bubble(affected)=o.collisionShrink*bubble(affected);
 if toc(timer)>o.maxOptimizationSeconds,info.code='time_limit';return;end
end
end
