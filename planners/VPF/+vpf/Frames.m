function frames=Frames(q,vehicle,alpha)
% Definition 1 and equation (9): length unchanged, width multiplied by alpha.
x=[vehicle.lw+vehicle.lf,vehicle.lw+vehicle.lf,-vehicle.lr,-vehicle.lr];
y=alpha*vehicle.lb/2*[1 -1 -1 1];
frames=cat(3,q(:,1)+cos(q(:,3))*x-sin(q(:,3))*y,q(:,2)+sin(q(:,3))*x+cos(q(:,3))*y);
end
