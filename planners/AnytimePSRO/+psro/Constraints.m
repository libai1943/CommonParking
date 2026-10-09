function data=Constraints(reference,c,o)
v=c.vehicle;body=[v.lw+v.lf v.lb/2;v.lw+v.lf -v.lb/2;-v.lr -v.lb/2;-v.lr v.lb/2];polygons=parking.PolygonData(c);N=numel(reference.x);
phase=(0:N-1)'/(N-1);rho=2*min(phase,1-phase);rho=rho/max(rho);
radius=v.vmax*(o.maximumStep+(1-o.maximumStep)*log1p(9*rho)/log(10));angle=repmat(v.phimax,N,1);
E=zeros(0,6);O=zeros(0,6);active=false(N,polygons.count);minimumExcess=inf;
for i=1:N
 pose=[reference.x(i) reference.y(i) reference.theta(i)];a=pose(3);R=[cos(a) -sin(a);sin(a) cos(a)];ego=body*R'+pose(1:2);box=psro.FootprintBox(pose,radius(i),angle(i),body);
 for j=1:polygons.count
  P=polygons.vertices{j};obsBox=[min(P,[],1),max(P,[],1)];
  if box(3)<obsBox(1)||box(1)>obsBox(3)||box(4)<obsBox(2)||box(2)>obsBox(4),continue;end
  active(i,j)=true;
  for k=1:4
   plane=psro.AreaPlane(ego(k,:),P,o.areaExcess);E(end+1,:)=[i,body(k,:),plane]; %#ok<AGROW>
   minimumExcess=min(minimumExcess,plane(1:2)*ego(k,:)'-plane(3)+o.areaExcess);
  end
  local=(P-pose(1:2))*R;
  for k=1:size(P,1)
   plane=psro.AreaPlane(local(k,:),body,o.areaExcess);O(end+1,:)=[i,P(k,:),plane]; %#ok<AGROW>
   minimumExcess=min(minimumExcess,plane(1:2)*local(k,:)'-plane(3)+o.areaExcess);
  end
 end
end
data=struct('ego_planes',E,'obstacle_planes',O,'position_radius',radius,'angle_radius',angle,'active_obstacles',active,'minimum_reference_area_excess',minimumExcess);
end
