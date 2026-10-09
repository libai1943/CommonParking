function initial=ManeuverSeed(delta,lateral,vehicle,options)
% Smooth local-frame initialization, not a Hybrid A* or geometric route.
N=options.primitiveIntervals;phase=linspace(0,1,N+1)';
if lateral==0
    length=max(4,2*abs(delta)/vehicle.kappa_max+2);amplitude=2*delta*vehicle.lw/length;
    alpha=amplitude*sin(pi*phase).^2;
    omega=amplitude*pi/length*sin(2*pi*phase);
    control=amplitude*2*pi^2/length^2*cos(2*pi*phase);
else
    length=8+2*abs(lateral);amplitude=.4*sign(lateral);
    alpha=amplitude*sin(pi*phase).^2.*sin(2*pi*phase);
    omega=amplitude*pi/length*(sin(2*pi*phase).^2+2*sin(pi*phase).^2.*cos(2*pi*phase));
    control=gradient(omega,length/N);
end
s=phase*length;theta=cumtrapz(s,tan(alpha)/vehicle.lw);x=cumtrapz(s,cos(theta));y=cumtrapz(s,sin(theta));
initial=struct('states',[x y theta alpha omega],'controls',control(1:end-1), ...
    'lengths',length,'phase',ones(N,1),'gear',1);
end
