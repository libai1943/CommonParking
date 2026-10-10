function phase=SolvePhase(reference,gear,c,o)
previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
phase=ritp.PathPhase(reference,gear,c,o);if ~phase.success,return;end
speed=ritp.Velocity(phase.problem.L,numel(phase.problem.s),c.vehicle,o);phase.velocity=speed;
if ~speed.success,phase.success=false;phase.code='velocity_qp_failed';return;end
% Convert the reference-length parameter to physical rear-axle velocity.
testTime=linspace(0,speed.T,max(2001,numel(speed.t)))';
testS=ritp.Basis(testTime,speed.T,5,0)*speed.coefficients;
testV=ritp.Basis(testTime,speed.T,5,1)*speed.coefficients;
testA=ritp.Basis(testTime,speed.T,5,2)*speed.coefficients;
g=ritp.Geometry(phase.coefficients,testS,phase.problem.L,gear,c.vehicle.lw);
if any(~isfinite([g.theta;g.phi;g.phi_s]))||min(g.parameter_speed)<o.derivativeTolerance
 phase.success=false;phase.code='singular_polynomial';return;
end
phase.maximum_steering=max(abs(g.phi));
if phase.maximum_steering>c.vehicle.phimax+o.feasibilityTolerance
 phase.success=false;phase.code='steering_bound_exceeded';return;
end
physicalV=gear*g.parameter_speed.*testV;
physicalA=gear*(g.parameter_speed.*testA+g.parameter_speed_derivative.*testV.^2);
physicalOmega=g.phi_s.*testV;
scale=max([1;max(abs(physicalV))/c.vehicle.vmax;sqrt(max(abs(physicalA))/c.vehicle.amax);max(abs(physicalOmega))/c.vehicle.wmax]);
if scale>1,scale=1.001*scale;end
phase.time_scale=scale;phase.duration=scale*speed.T;
t=unique([speed.t;linspace(0,speed.T,ceil(scale*speed.T/.02)+1)']);
% Nearly equal grid points can round to an identical time after adding the
% preceding phase duration. Merge them before concatenation, retaining tf.
t=t([true;diff(t)>1e-9]);t(end)=speed.T;
s=ritp.Basis(t,speed.T,5,0)*speed.coefficients;
ds=ritp.Basis(t,speed.T,5,1)*speed.coefficients/scale;
dds=ritp.Basis(t,speed.T,5,2)*speed.coefficients/scale^2;
g=ritp.Geometry(phase.coefficients,s,phase.problem.L,gear,c.vehicle.lw);
angleShift=2*pi*round((reference(1,3)-g.theta(1))/(2*pi));g.theta=g.theta+angleShift;
phase.trajectory=struct('t',scale*t,'x',g.x,'y',g.y,'theta',g.theta, ...
 'v',gear*g.parameter_speed.*ds,'phi',g.phi, ...
 'a',gear*(g.parameter_speed.*dds+g.parameter_speed_derivative.*ds.^2),'omega',g.phi_s.*ds);
end
