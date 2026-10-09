function primitives=Shortest(connection,a,b,kappa)
% Equal forward/reverse costs: the obstacle-free shortest Reeds-Shepp curve.
[paths,costs]=connect(connection,a,b);assert(~isempty(paths)&&isfinite(costs(1)),'No Reeds-Shepp connection.');p=paths{1};
curvature=zeros(numel(p.MotionLengths),1);
for j=1:numel(curvature)
 if strcmp(p.MotionTypes{j},'L'),curvature(j)=kappa;elseif strcmp(p.MotionTypes{j},'R'),curvature(j)=-kappa;end
end
primitives=[p.MotionLengths(:).*p.MotionDirections(:),curvature];primitives(abs(primitives(:,1))<1e-11,:)=[];
end

