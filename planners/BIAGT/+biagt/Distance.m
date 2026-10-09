function d=Distance(q,Q,scale)
z=Q-q;z(:,3)=atan2(sin(z(:,3)),cos(z(:,3)));d=sqrt(sum((z.*scale).^2,2));
end
