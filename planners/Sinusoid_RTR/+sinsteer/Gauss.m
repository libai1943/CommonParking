function [z,w]=Gauss(n)
persistent nodes weights
if isempty(nodes),nodes=cell(1,64);weights=nodes;end
if isempty(nodes{n}),j=(1:n-1)';b=j./sqrt(4*j.^2-1);[V,D]=eig(diag(b,1)+diag(b,-1));[z,ix]=sort(diag(D));nodes{n}=(z+1)/2;weights{n}=V(1,ix)'.^2;end
z=nodes{n};w=weights{n};
end
