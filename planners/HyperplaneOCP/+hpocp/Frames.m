function frames=Frames(q,v)
x=[v.lw+v.lf,v.lw+v.lf,-v.lr,-v.lr];y=v.lb/2*[1 -1 -1 1];
frames=cat(3,q(:,1)+cos(q(:,3))*x-sin(q(:,3))*y,q(:,2)+sin(q(:,3))*x+cos(q(:,3))*y);
end
