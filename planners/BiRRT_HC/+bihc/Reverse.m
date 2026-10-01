function controls=Reverse(controls)
% Reverse traversal: physical heading/curvature unchanged, gear negated.
controls=flipud(controls);
controls(:,2)=controls(:,2)+controls(:,3).*abs(controls(:,1));
controls(:,1)=-controls(:,1);controls(:,3)=-controls(:,3);
end
