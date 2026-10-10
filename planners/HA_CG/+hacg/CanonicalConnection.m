function [primitives,found]=CanonicalConnection(start,goal,data,margin,spacing)
% Serialize an accepted goal connection using the reference Toolbox order.
% This is called only after the numeric candidate screen finds a free arc,
% not at every expansion. It prevents roundoff differences in equivalent
% Reeds--Shepp words from perturbing the subsequent nonlinear CG iterations.
v=data.vehicle;rs=reedsSheppConnection('MinTurningRadius',v.turning_radius_min,'ReverseCost',1);
[paths,costs]=connect(rs,start,goal,'PathSegments','all');[~,order]=sort(costs(:));
found=false;primitives=zeros(0,2);
for j=order'
 if ~isfinite(costs(j)),continue;end
 p=paths{j};curvature=zeros(numel(p.MotionLengths),1);
 for k=1:numel(curvature)
  if strcmp(p.MotionTypes{k},'L'),curvature(k)=v.kappa_max;end
  if strcmp(p.MotionTypes{k},'R'),curvature(k)=-v.kappa_max;end
 end
 pp=[p.MotionLengths(:).*p.MotionDirections(:),curvature];pp(abs(pp(:,1))<1e-10,:)=[];
 if hacg.FootprintFree(parking.SamplePrimitives(start,pp,spacing),data,margin)
  primitives=pp;found=true;return;
 end
end
end
