function report=TestRITP()
c=LoadCase(1);v=c.vehicle;o=ritp.Config();S=linspace(0,5,51)';reference=[S,zeros(size(S)),zeros(size(S))];
[coeff,solver,problem]=ritp.PathQP(reference,1,ones(size(S)),o);assert(solver.success);
assert(max(abs(problem.Aeq*coeff(:)-problem.beq))<1e-8);
g=ritp.Geometry(coeff,problem.s,problem.L,1,v.lw);assert(max(abs(g.y))<1e-8&&max(abs(g.phi))<1e-8);
z=coeff(:);cost=sum(o.pathWeights(1)*sum((ritp.Basis(S,5,o.degree,0)*coeff-reference(:,1:2)).^2,2));
for order=1:2,cost=cost+o.pathWeights(order+1)*sum((ritp.Basis(problem.s,5,o.degree,order)*coeff).^2,'all');end
qpCost=.5*z'*problem.H*z+problem.f'*z+o.pathWeights(1)*sum(reference(:,1:2).^2,'all');
costError=abs(cost-qpCost);assert(costError<1e-8);
speed=ritp.Velocity(5,numel(problem.s),v,o);assert(speed.success);
quinticError=max(abs(speed.coefficients-[0;0;0;50;-75;30]));assert(quinticError<1e-8);
assert(max(abs(speed.v))<=v.vmax&&max(abs(speed.a))<=v.amax);
% A plus-shaped crossing has no contained vertices; the adapter uses SAT.
c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[1 2 2 1 1],'y',[-4 -4 4 4 -4])};
assert(~parking.FootprintClearance([0 0 0],c,0));
% Common endpoint tangents constrain both coordinates and reverse gear.
reference=[S,0.3*sin(pi*S/5),repmat(pi/3,size(S))];
[coeff,solver,problem]=ritp.PathQP(reference,1,ones(size(S)),o);assert(solver.success);
g=ritp.Geometry(coeff,[0;5],5,1,v.lw);headingError=atan2(sin(g.theta-reference([1 end],3)),cos(g.theta-reference([1 end],3)));
assert(max(abs(headingError))<1e-8);
[reverse,ok,~]=ritp.PathQP(reference,-1,ones(size(S)),o);assert(ok.success);
gr=ritp.Geometry(reverse,[0;5],5,-1,v.lw);
assert(max(abs(atan2(sin(gr.theta-reference([1 end],3)),cos(gr.theta-reference([1 end],3)))))<1e-8);
% Analytic steering derivative versus centered finite differences.
testCoefficients=[0 0;3 1;.3 -.2;-.2 .4;.1 -.1;.05 .03];ss=1.4;h=1e-5;
g=ritp.Geometry(testCoefficients,ss,3,-1,v.lw);left=ritp.Geometry(testCoefficients,ss-h,3,-1,v.lw);right=ritp.Geometry(testCoefficients,ss+h,3,-1,v.lw);
derivativeError=abs(g.phi_s-(right.phi-left.phi)/(2*h));assert(derivativeError<1e-8);
% Independent finite differences verify the physical chain rule, including
% a non-unit polynomial tangent and reverse motion.
rate=.63;accel=.17;timeStep=1e-5;
left=ritp.Geometry(testCoefficients,ss-rate*timeStep+.5*accel*timeStep^2,3,-1,v.lw);
right=ritp.Geometry(testCoefficients,ss+rate*timeStep+.5*accel*timeStep^2,3,-1,v.lw);
velocity=-g.parameter_speed*rate;
rhs=[velocity*cos(g.theta),velocity*sin(g.theta),velocity*tan(g.phi)/v.lw,g.phi_s*rate];
fd=([right.x,right.y,right.theta,right.phi]-[left.x,left.y,left.theta,left.phi])/(2*timeStep);
chainError=max(abs(fd-rhs));assert(chainError<1e-7);
acceleration=-(g.parameter_speed*accel+g.parameter_speed_derivative*rate^2);
fdAcceleration=(-right.parameter_speed*(rate+accel*timeStep)+left.parameter_speed*(rate-accel*timeStep))/(2*timeStep);
assert(abs(fdAcceleration-acceleration)<1e-7);
c.obstacle.num_obs=0;c.obstacle.obs={};minimumTimeStep=Inf;
for L=[.5 1 2 3 4 5 7 10]
 s=linspace(0,L,31)';phase=ritp.Phase([s,zeros(size(s)),zeros(size(s))],1,c,o);assert(phase.success);
 minimumTimeStep=min(minimumTimeStep,min(diff(phase.trajectory.t+1000)));
end
assert(minimumTimeStep>1e-10);
report=struct('passed',true,'path_objective_error',costError,'quintic_closed_form_error',quinticError, ...
 'steering_derivative_error',derivativeError,'chain_rule_error',chainError,'acceleration_chain_rule_error',abs(fdAcceleration-acceleration), ...
 'crossing_rectangles_detected_by_adapter',true,'endpoint_heading_error_rad',headingError,'minimum_shifted_time_step',minimumTimeStep);disp(report);
end
