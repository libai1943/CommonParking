function free=ArcsFree(q,primitives,c)
% Section III-B2: analytic vertex-arc/edge contacts, also with roles reversed.
% Straight segments use the exact swept convex polygon. No pose sampling.
free=false;polygons=parking.PolygonData(c);
if ~parking.FootprintClearance(q,c,0),return;end
for primitive=1:size(primitives,1)
 length=primitives(primitive,1);curvature=primitives(primitive,2);
 if abs(length)<1e-12,continue;end
 next=parking.IntegratePrimitive(q,length,curvature);body=parking.VehiclePolygon(q,c.vehicle);body=body(1:4,:);
 if abs(curvature)<1e-12
  other=parking.VehiclePolygon(next,c.vehicle);points=[body;other(1:4,:)];index=convhull(points(:,1),points(:,2));sweep=points(index(1:end-1),:);
  for o=1:polygons.count,if laumond.PolygonsIntersect(sweep,polygons.vertices{o}),return;end,end
 else
  centre=q(1:2)+[-sin(q(3)),cos(q(3))]/curvature;turn=length*curvature;
  for o=1:polygons.count
   obstacle=polygons.vertices{o};
   for vertex=1:size(body,1)
    for edge=1:size(obstacle,1)
     if arcSegment(body(vertex,:),centre,turn,obstacle(edge,:),obstacle(mod(edge,size(obstacle,1))+1,:)),return;end
    end
   end
   % An obstacle vertex crossing a moving body edge becomes a backwards
   % rotating obstacle point crossing the fixed initial body edge.
   for vertex=1:size(obstacle,1)
    for edge=1:size(body,1)
     if arcSegment(obstacle(vertex,:),centre,-turn,body(edge,:),body(mod(edge,size(body,1))+1,:)),return;end
    end
   end
  end
 end
 q=next;
end
free=true;
end
function hit=arcSegment(start,centre,turn,a,b)
hit=false;radial=start-centre;d=b-a;offset=a-centre;A=dot(d,d);B=2*dot(offset,d);C=dot(offset,offset)-dot(radial,radial);
disc=B*B-4*A*C;tolerance=1e-11*max(1,B*B+abs(4*A*C));
if disc < -tolerance,return;end
roots=(-B+[-1 1]*sqrt(max(0,disc)))/(2*A);
for t=roots
 if t < -1e-10||t>1+1e-10,continue;end
 point=a+max(0,min(1,t))*d-centre;
 angle=atan2(radial(1)*point(2)-radial(2)*point(1),dot(radial,point));
 if abs(angle)<1e-10||mod(sign(turn)*angle,2*pi)<=abs(turn)+1e-10,hit=true;return;end
end
end
