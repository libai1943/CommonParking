function report=TestLatticePrimitives(file)
stored=load(file,'library');library=stored.library;p=library.primitives;N=numel(p);assert(N==480);
v=library.vehicle;z=zeros(N,5);goal=z;gear=zeros(N,1);step=zeros(N,1);controls=zeros(N,60);
for j=1:N
    item=p{j};assert(numel(item.controls)==60);assert(item.cost>=item.length-1e-8);
    z(j,:)=item.states(1,:);goal(j,:)=item.states(end,:);gear(j)=item.gear;step(j)=item.length/60;controls(j,:)=item.controls';
    assert(norm(item.states(end,1:2)-item.delta)<1e-7);
    angle=item.states([1 end],3)-library.options.headings([item.from item.to])';assert(max(abs(atan2(sin(angle),cos(angle))))<1e-7);
    assert(max(abs(item.states([1 end],4:5)),[],'all')<1e-8);
    if strcmp(item.kind,'parallel')
        angle=item.states(1,3);assert(abs(item.delta*[-sin(angle);cos(angle)])>.1,'Side-shift primitive collapsed to a straight line.');
    end
end
% Independently integrate the ORIGINAL five-state ODE with 16 RK4 substeps
% per shooting interval, including conventional alpha and omega propagation.
peakAlpha=max(abs(z(:,4)));peakRate=max(abs(z(:,5)));cost=zeros(N,1);h=step/16;
for interval=1:60
    u=controls(:,interval);
    for sub=1:16
        a=ode(z,u);b=ode(z+.5*h.*a,u);cc=ode(z+.5*h.*b,u);d=ode(z+h.*cc,u);
        cost=cost+h/6.*(integrand(z,u)+2*integrand(z+.5*h.*a,u)+2*integrand(z+.5*h.*b,u)+integrand(z+h.*cc,u));
        z=z+h/6.*(a+2*b+2*cc+d);peakAlpha=max(peakAlpha,max(abs(z(:,4))));peakRate=max(peakRate,max(abs(z(:,5))));
    end
end
error=max(abs(z-goal),[],'all');original=cellfun(@(x)x.cost,p);costError=max(abs(cost-original));
assert(error<1e-4&&costError<1e-3,'Offline discretization is too inaccurate.');
assert(peakAlpha<=v.phimax+1e-3&&peakRate<=library.options.steeringRatePerMetre+1e-6);
report=struct('passed',true,'primitives',N,'independent_integration_error',error,'objective_quadrature_error',costError,'peak_steering',peakAlpha,'peak_spatial_steering_rate',peakRate);disp(report);
    function f=ode(state,u)
        f=[gear.*cos(state(:,3)),gear.*sin(state(:,3)),gear.*tan(state(:,4))/v.lw,state(:,5),u];
    end
    function value=integrand(state,u)
        value=1+library.options.gamma*(state(:,4).^2+10*state(:,5).^2+u.^2);
    end
end
