function [primitives,ok,tested]=Connect(connection,a,b,c,o)
% Footnote 1 explicitly makes imax=1 the shortest RS path.
[paths,costs]=connect(connection,a,b,'PathSegments','all');[~,order]=sort(costs(:));
primitives=zeros(0,2);ok=false;tested=0;
for j=1:min(o.maximumRSPaths,numel(order))
 index=order(j);if ~isfinite(costs(index)),break;end
 p=paths{index};curvature=zeros(numel(p.MotionLengths),1);
 for k=1:numel(curvature)
  if strcmp(p.MotionTypes{k},'L'),curvature(k)=c.vehicle.kappa_max;elseif strcmp(p.MotionTypes{k},'R'),curvature(k)=-c.vehicle.kappa_max;end
 end
 trial=[p.MotionLengths(:).*p.MotionDirections(:),curvature];trial(abs(trial(:,1))<1e-11,:)=[];tested=tested+1;
 if ~isempty(trial)&&wgrrt.ArcsFree(a,trial,c),primitives=trial;ok=true;return;end
end
end
