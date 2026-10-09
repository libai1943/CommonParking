function controls=Reverse(controls)
% Reverse traversal of cubic spirals, preserving physical curvature.
controls=flipud(controls);L=abs(controls(:,1));
controls(:,2)=controls(:,2)+controls(:,3).*L+.5*controls(:,4).*L.^2;
controls(:,3)=-controls(:,3)-controls(:,4).*L;
controls(:,1)=-controls(:,1);
end
