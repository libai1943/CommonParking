function [plane,signs]=AreaPlane(point,polygon,excess)
% Sum of signed triangle areas with the signs frozen at the reference point.
next=polygon([2:end 1],:);terms=.5*[polygon(:,2)-next(:,2),next(:,1)-polygon(:,1),polygon(:,1).*next(:,2)-next(:,1).*polygon(:,2)];
areas=terms*[point(:);1];signs=ones(size(areas));signs(areas<0)=-1;
coeff=signs'*terms;plane=[coeff(1:2),polyarea(polygon(:,1),polygon(:,2))+excess-coeff(3)];
end
