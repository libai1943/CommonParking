function [alpha,info]=ProtectionAlpha(states,controls,h,vehicle,o)
kappa=tan(controls(:,2))/vehicle.lw;velocity=states(1:end-1,4);acceleration=controls(:,1);
turn=(velocity*h+.5*acceleration*h^2).*kappa;
info=struct('success',false,'code','turn_outside_lemma','maximum_turn',max(abs(turn)),'internal_speed_reversals',nnz(velocity.*(velocity+h*acceleration)<0));alpha=NaN;
if any(abs(turn)>=pi),return;end
left=1;right=1;
while any(vpf.CoverDistance(right,velocity,acceleration,kappa,h,vehicle)>1e-12,'all')
 right=1+2*(right-1+.001);
 if right>o.maximumAlpha,info.code='alpha_bracket_failed';return;end
end
for j=1:60
 middle=(left+right)/2;
 if all(vpf.CoverDistance(middle,velocity,acceleration,kappa,h,vehicle)<=0,'all'),right=middle;else,left=middle;end
end
alpha=right;info.success=true;info.code='alpha_found';info.maximum_cover_distance=max(vpf.CoverDistance(alpha,velocity,acceleration,kappa,h,vehicle),[],'all');
end
