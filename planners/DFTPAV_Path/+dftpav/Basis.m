function b=Basis(t,d)
% Degree-five power basis and its exact d-th derivative.
t=t(:);b=zeros(numel(t),6);
for j=d:5,b(:,j+1)=prod(j-d+1:j)*t.^(j-d);end
end
