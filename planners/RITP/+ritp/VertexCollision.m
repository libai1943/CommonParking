function [index,obstacle]=VertexCollision(pose,c)
% Literal equation (2): mutual vertex containment, NOT a full SAT test.
index=0;obstacle=0;v=c.vehicle;polygons=parking.PolygonData(c);
body=[v.lw+v.lf v.lb/2;v.lw+v.lf -v.lb/2;-v.lr -v.lb/2;-v.lr v.lb/2];
for j=1:size(pose,1)
 angle=pose(j,3);R=[cos(angle) -sin(angle);sin(angle) cos(angle)];ego=body*R'+pose(j,1:2);
 for k=1:polygons.count
  obs=polygons.vertices{k};
  if any(inpolygon(ego(:,1),ego(:,2),obs(:,1),obs(:,2)))||any(inpolygon(obs(:,1),obs(:,2),ego(:,1),ego(:,2)))
   index=j;obstacle=k;return;
  end
 end
end
end
