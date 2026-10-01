function [nodes,D,weights] = Radau3()
% Three-stage right Radau collocation, including an initial interpolation node.
nodes=[0;(4-sqrt(6))/10;(4+sqrt(6))/10;1];D=zeros(3,4);weights=zeros(3,1);
for j=1:4
    others=nodes([1:j-1 j+1:4]);basis=poly(others)/prod(nodes(j)-others);
    D(:,j)=polyval(polyder(basis),nodes(2:4));
end
for j=1:3
    others=nodes(setdiff(2:4,j+1));basis=poly(others)/prod(nodes(j+1)-others);
    integral=polyint(basis);weights(j)=polyval(integral,1)-polyval(integral,0);
end
end
