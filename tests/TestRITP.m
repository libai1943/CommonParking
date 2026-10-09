function report=TestRITP()
c=LoadCase(1);v=c.vehicle;o=ritp.Config();S=linspace(0,5,51)';reference=[S,zeros(size(S)),zeros(size(S))];
[coeff,solver,problem]=ritp.PathQP(reference,1,ones(size(S)),o);assert(solver.success);
assert(max(abs(problem.Aeq*coeff(:)-problem.beq))<1e-8);
g=ritp.Geometry(coeff,problem.s,problem.L,1,v.lw);assert(max(abs(g.y))<1e-8&&max(abs(g.phi))<1e-8);
z=coeff(:);cost=sum(o.pathWeights(1)*sum((ritp.Basis(S,5,5,0)*coeff-reference(:,1:2)).^2,2));
for order=1:2,cost=cost+o.pathWeights(order+1)*sum((ritp.Basis(problem.s,5,5,order)*coeff).^2,'all');end
qpCost=.5*z'*problem.H*z+problem.f'*z+o.pathWeights(1)*sum(reference(:,1:2).^2,'all');
costError=abs(cost-qpCost);assert(costError<1e-8);
speed=ritp.Velocity(5,numel(problem.s),v,o);assert(speed.success);
quinticError=max(abs(speed.coefficients-[0;0;0;50;-75;30]));assert(quinticError<1e-8);
assert(max(abs(speed.v))<=v.vmax&&max(abs(speed.a))<=v.amax);
% Paper equation (2) misses a plus-shaped rectangle intersection.
c.obstacle.num_obs=1;c.obstacle.obs={struct('x',[1 2 2 1 1],'y',[-4 -4 4 4 -4])};
[index,~]=ritp.VertexCollision([0 0 0],c);assert(index==0);
% Equation (25)'s x-only alignment does not constrain the y tangent.
reference=[S,0.3*sin(pi*S/5),repmat(pi/3,size(S))];
[coeff,solver,problem]=ritp.PathQP(reference,1,ones(size(S)),o);assert(solver.success);
g=ritp.Geometry(coeff,[0;5],5,1,v.lw);headingError=atan2(sin(g.theta-reference([1 end],3)),cos(g.theta-reference([1 end],3)));
assert(max(abs(headingError))>.1);
% Analytic steering derivative versus centered finite differences.
testCoefficients=[0 0;3 1;.3 -.2;-.2 .4;.1 -.1;.05 .03];ss=1.4;h=1e-5;
g=ritp.Geometry(testCoefficients,ss,3,-1,v.lw);left=ritp.Geometry(testCoefficients,ss-h,3,-1,v.lw);right=ritp.Geometry(testCoefficients,ss+h,3,-1,v.lw);
derivativeError=abs(g.phi_s-(right.phi-left.phi)/(2*h));assert(derivativeError<1e-8);
report=struct('passed',true,'path_objective_error',costError,'quintic_closed_form_error',quinticError, ...
 'steering_derivative_error',derivativeError,'vertex_test_misses_crossing_rectangles',true,'x_only_tsc_heading_error_rad',headingError);disp(report);
end
