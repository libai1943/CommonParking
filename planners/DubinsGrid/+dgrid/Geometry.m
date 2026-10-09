function [pieces,objective,inequality]=Geometry(z,reference,c,centres,bounds)
% Section 4.2: scalar Frenet offsets, Menger curvature and body heading.
polygons=parking.PolygonData(c);pieces=reference;terms=zeros(1,3);inequality=[];
for j=1:numel(reference)
 r=reference{j};chi=zeros(size(r.q,1),1);chi(r.variable_nodes)=z(r.variables);
 xy=r.q(:,1:2)+chi.*[-sin(r.q(:,3)),cos(r.q(:,3))];n=size(xy,1);
 a=xy(2:end-1,:)-xy(1:end-2,:);b=xy(3:end,:)-xy(2:end-1,:);chord=xy(3:end,:)-xy(1:end-2,:);
 den=max(1e-12,hypot(a(:,1),a(:,2)).*hypot(b(:,1),b(:,2)).*hypot(chord(:,1),chord(:,2)));
 signed=2*(a(:,1).*b(:,2)-a(:,2).*b(:,1))./den;kap=r.gear*[signed(1);signed;signed(end)];
 tangent=[xy(2,:)-xy(1,:);chord;xy(end,:)-xy(end-1,:)];theta=unwrap(atan2(r.gear*tangent(:,2),r.gear*tangent(:,1)));
 theta=theta+2*pi*round((r.q(1,3)-theta(1))/(2*pi));theta(1)=r.q(1,3);theta(end)=r.q(end,3)+2*pi*round((theta(end)-r.q(end,3))/(2*pi));
 lengths=hypot(diff(xy(:,1)),diff(xy(:,2)));q=[xy,theta];gap=dgrid.DiscGap(q,polygons.vertices,centres,c.vehicle.lb/2,bounds);
 terms=terms+[sum(abs(diff(abs(kap)))),sum(abs(atan2(sin(diff(theta)),cos(diff(theta))))),sum(chi.^2)];
 inequality=[inequality;abs(kap)-c.vehicle.kappa_max;-gap(:);1e-6-lengths]; %#ok<AGROW>
 pieces{j}=struct('q',q,'kappa',kap,'s',[0;cumsum(lengths)],'gear',r.gear,'offset',chi);
end
objective=sum(terms);
end
