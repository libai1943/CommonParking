function d=CoverDistance(alpha,velocity,acceleration,kappa,h,vehicle)
% Equations (11)-(15), algebraically stabilized near zero curvature.
x=[vehicle.lw+vehicle.lf,vehicle.lw+vehicle.lf,vehicle.lr,vehicle.lr];
y=vehicle.lb/2*[1 -1 -1 1];signs=[1 -1 -1 1];
turn=(velocity*h+.5*acceleration*h^2).*kappa;
rho=sqrt((1-kappa.*y).^2+(kappa.*x).^2);
expanded=sqrt((1-alpha.*kappa.*y).^2+(kappa.*x).^2);
radial=(-2*(alpha-1).*y+(alpha.^2-1).*kappa.*y.^2)./(expanded+rho);
sagitta=zeros(size(kappa));active=abs(kappa)>1e-14;sagitta(active)=2*sin(turn(active)/4).^2./kappa(active);
d=signs.*(radial-sagitta.*expanded);
end
