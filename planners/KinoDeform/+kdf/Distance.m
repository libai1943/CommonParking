function d=Distance(Q,q,scale)
e=Q-q;e(:,3)=atan2(sin(e(:,3)),cos(e(:,3)));d=max(abs(e).*scale,[],2);
end
