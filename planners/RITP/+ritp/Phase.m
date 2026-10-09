function phase=Phase(reference,gear,c,o)
previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
phase=ritp.PathPhase(reference,gear,c,o);if ~phase.success,return;end
speed=ritp.Velocity(phase.problem.L,numel(phase.problem.s),c.vehicle,o);phase.velocity=speed;
if ~speed.success,phase.success=false;phase.code='velocity_qp_failed';return;end
g=ritp.Geometry(phase.coefficients,speed.s,phase.problem.L,gear,c.vehicle.lw);
if any(~isfinite([g.theta;g.phi;g.phi_s]))||min(g.parameter_speed)<o.derivativeTolerance
 phase.success=false;phase.code='singular_polynomial';return;
end
angleShift=2*pi*round((reference(1,3)-g.theta(1))/(2*pi));g.theta=g.theta+angleShift;
phase.trajectory=struct('t',speed.t,'x',g.x,'y',g.y,'theta',g.theta,'v',gear*speed.v,'phi',g.phi,'a',gear*speed.a,'omega',g.phi_s.*speed.v);
end
