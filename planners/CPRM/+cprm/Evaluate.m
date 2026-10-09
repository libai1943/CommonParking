function [q,speed,first]=Evaluate(piece,t)
t=t(:);
if strcmp(piece.type,'arc')
 q=parking.IntegratePrimitive(piece.start,piece.primitive(1)*t,piece.primitive(2));speed=repmat(abs(piece.primitive(1)),size(t));
 first=piece.direction*speed.*[cos(q(:,3)),sin(q(:,3))];q(:,4)=piece.primitive(2);return;
end
c=piece.coefficients;p=[ones(size(t)),t,t.^2,t.^3]*c;first=[zeros(size(t)),ones(size(t)),2*t,3*t.^2]*c;
second=[zeros(size(t)),zeros(size(t)),2*ones(size(t)),6*t]*c;speed=hypot(first(:,1),first(:,2));
theta=atan2(piece.direction*first(:,2),piece.direction*first(:,1));kappa=piece.direction*(first(:,1).*second(:,2)-first(:,2).*second(:,1))./speed.^3;q=[p theta kappa];
end
