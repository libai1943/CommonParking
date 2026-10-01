function profile = LongitudinalProfile(length,vmax,accel,brake)
% Exact rest-to-rest minimum-time solution of the bounded 1-D double integrator.
peak = min(vmax,sqrt(2*length/(1/accel+1/brake)));
ta = peak/accel; td = peak/brake;
sa = peak^2/(2*accel); sd = peak^2/(2*brake);
tc = max(0,(length-sa-sd)/peak);
profile=struct('length',length,'peak',peak,'accel',accel,'brake',brake, ...
    'ta',ta,'tc',tc,'td',td,'sa',sa,'sd',sd,'tf',ta+tc+td);
end
