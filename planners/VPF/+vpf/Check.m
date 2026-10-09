function report=Check(c,z,u,h,alpha,o)
prediction=vpf.Step(z(1:end-1,:),u,h,c.vehicle.lw);dynamics=max(abs(prediction-z(2:end,:)),[],'all');
t=c.task;boundary=[z(1,:)-[t.x0 t.y0 t.theta0 0];z(end,:)-[t.xf t.yf t.thetaf 0]];boundary(:,3)=atan2(sin(boundary(:,3)),cos(boundary(:,3)));
v=c.vehicle;violations=[abs(z(:,4))-v.vmax;abs(u(:,1))-v.amax;abs(u(:,2))-v.phimax; ...
 abs(diff(u(:,1)))/h-o.jerkLimit;abs(diff(u(:,2)))/h-v.wmax;o.minimumTime-h*size(u,1);h*size(u,1)-o.maximumTime];
[A,B]=vpf.Segments(vpf.Frames(z(:,1:3),v,alpha));polygons=parking.PolygonData(c);minimum=inf;
for j=1:polygons.count
 P=polygons.vertices{j};
 for k=1:size(P,1),minimum=min(minimum,min(vpf.SegmentScore(A,B,P(k,:),P(mod(k,size(P,1))+1,:))));end
end
violation=max([0;violations;o.segmentThreshold-minimum]);endpoint=max(abs(boundary),[],'all');
report=struct('success',dynamics<=o.feasibilityTolerance&&endpoint<=o.feasibilityTolerance&&violation<=o.feasibilityTolerance, ...
 'dynamics_error',dynamics,'boundary_error',endpoint,'maximum_inequality_violation',violation,'minimum_segment_score',minimum,'tf',h*size(u,1));
end
