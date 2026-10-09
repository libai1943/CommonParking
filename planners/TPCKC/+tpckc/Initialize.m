function [initial,details]=Initialize(path,c,o)
seed=cp.EmptyResult('TPCKC_seed',c.id,'path');seed.path=path;ref=cpe.MakeReference(seed,c);N=ceil(ref.tf/o.resamplingTime);
assert(N<=o.maximumIntervals,'Initialization exceeds the disclosed interval resource cap.');scale=N*o.resamplingTime/ref.tf;
time=(0:N)'*o.resamplingTime;query=time/scale;initial=cpe.ReferenceAt(ref,query,c.vehicle);
% Fit time-parameterized splines separately within each direction run. All
% raw path cusps are spline endpoints, never averaged across a reversal.
for j=1:numel(ref.profiles)
 ids=path.cusp_indices(j):path.cusp_indices(j+1);s=path.s(ids)-path.s(ids(1));profile=ref.profiles{j};knots=zeros(size(s));
 accelerating=s<=profile.sa;decelerating=s>=profile.length-profile.sd;cruising=~accelerating&~decelerating;
 knots(accelerating)=sqrt(2*s(accelerating)/profile.accel);knots(cruising)=profile.ta+(s(cruising)-profile.sa)/profile.peak;
 knots(decelerating)=profile.tf-sqrt(max(0,2*(profile.length-s(decelerating))/profile.brake));knots=knots+ref.times(j);
 mask=query>=ref.times(j)&query<=ref.times(j+1)+1e-12;
 for field={'x','y','theta'},name=field{1};initial.(name)(mask)=spline(knots,path.(name)(ids),query(mask));end
end
initial.t=time;initial.v=initial.v/scale;initial.a=initial.a/scale^2;initial.phi(:)=0;initial.omega(:)=0;
details=struct('intervals',N,'original_pmp_time_s',ref.tf,'initial_time_s',time(end),'time_rounding_scale',scale,'spline_per_gear_run',true,'steering_initialized_zero',true);
end
