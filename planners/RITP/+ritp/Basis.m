function B=Basis(s,L,degree,order)
% Normalized monomials, differentiated with respect to physical parameter s.
u=s(:)/L;B=zeros(numel(u),degree+1);
for k=order:degree
 coefficient=prod((k-order+1):k);if order==0,coefficient=1;end
 B(:,k+1)=coefficient*u.^(k-order)/L^order;
end
end
