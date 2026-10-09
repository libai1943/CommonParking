function candidates=Maintain(candidates,c,env,o)
if isempty(candidates),return;end
% Retain the lowest-cost representative if every candidate contains a detour.
[~,order]=sort([candidates.cost]);candidates=candidates(order);keep=false(size(candidates));signatures=[];
delta=[c.task.xf-c.task.x0,c.task.yf-c.task.y0];direction=delta/max(norm(delta),1e-12);
for k=1:numel(candidates)
 b=candidates(k);signature=teb.Signature(b.pose(:,1:2),env);d=diff(b.pose(:,1:2));lengths=vecnorm(d,2,2);
 forward=all((d*direction')./max(lengths,1e-12)>o.forwardCosine);
 if (~any(keep)||forward)&&isfinite(signature)&&all(abs(signatures-signature)>o.signatureTolerance)
  keep(k)=true;signatures(end+1)=signature;candidates(k).signature=signature; %#ok<AGROW>
 end
end
candidates=candidates(keep);
end
