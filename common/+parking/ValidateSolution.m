function report=ValidateSolution(c,r)
% Independent dense replay, exact terminal pose, full-body swept clearance.
% A Lipschitz bound bridges samples for each constant-curvature primitive.
t=c.task; start=[t.x0 t.y0 t.theta0]; goal=[t.xf t.yf t.thetaf];
[q,d,k,s]=parking.SamplePrimitives(start,r.primitives,0.01);
[~,gap]=parking.FootprintClearance(q,c);
rho=hypot(c.vehicle.lw+c.vehicle.lf,c.vehicle.lb/2);
motionBound=abs(s).*(1+rho*abs(k))/2;
continuousLower=min(min(gap(1:end-1),gap(2:end))-motionBound);
report.path_length=sum(abs(s));
report.forward_length=sum(s(s>0)); report.reverse_length=-sum(s(s<0));
report.gear_changes=sum(diff(d)~=0);
report.position_error=norm(q(end,1:2)-goal(1:2));
report.heading_error=abs(atan2(sin(q(end,3)-goal(3)),cos(q(end,3)-goal(3))));
report.max_abs_curvature=max(abs(k));
report.sample_clearance_lower_bound=min(gap);
report.continuous_clearance_lower_bound=continuousLower;
report.feasible=r.success && report.position_error<1e-7 && report.heading_error<1e-7 ...
    && report.max_abs_curvature<=c.vehicle.kappa_max+1e-10 && continuousLower>0;
% Unconstrained Dubins distances bound ALL obstacle-avoiding one-direction paths.
dc=dubinsConnection('MinTurningRadius',c.vehicle.turning_radius_min);
[~,forward]=connect(dc,start,goal);
backStart=start;backGoal=goal;backStart(3)=backStart(3)+pi;backGoal(3)=backGoal(3)+pi;
[~,reverse]=connect(dc,backStart,backGoal);
report.forward_only_lower_bound=forward(1);
report.reverse_only_lower_bound=reverse(1);
report.single_direction_lower_bound=min(forward(1),reverse(1));
report.mixed_optimality_gap=report.single_direction_lower_bound-report.path_length;
report.length_optimum_requires_both_directions=report.feasible ...
    && report.forward_length>0.1 && report.reverse_length>0.1 && report.mixed_optimality_gap>1e-5;
end
