function value=Signature(xy,env)
% Equations (20)-(22): minimum-absolute branch with its SIGN retained.
z=complex(xy(:,1),xy(:,2));value=0;
for j=1:numel(env.markers)
 d=z-env.markers(j);if any(abs(d)<1e-10),value=NaN;return;end
 angles=diff(angle(d));angles=atan2(sin(angles),cos(angles));
 value=value+env.residues(j)*(log(abs(d(end)))-log(abs(d(1)))+1i*sum(angles));
end
end
