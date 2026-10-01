# Obstacle-free, full-horizon tracking by Hermite-Simpson transcription.
param N integer >= 3;
param Tref > 0;
param L > 0;
param V > 0;
param A > 0;
param P > 0;
param W > 0;
param Tmin > 0;
param Tmax > Tmin;
param time_weight >= 0;
param smooth_weight >= 0;
param q0{1..3};
param ref{1..N,1..5};
var tf >= Tmin, <= Tmax;
var x{1..N}; var y{1..N}; var theta{1..N};
var v{1..N} >= -V, <= V;
var phi{1..N} >= -P, <= P;
var a{1..N} >= -A, <= A;
var w{1..N} >= -W, <= W;
var vm{i in 1..N-1} = (v[i]+v[i+1])/2+tf/(N-1)/8*(a[i]-a[i+1]);
var pm{i in 1..N-1} = (phi[i]+phi[i+1])/2+tf/(N-1)/8*(w[i]-w[i+1]);
var tm{i in 1..N-1} = (theta[i]+theta[i+1])/2+tf/(N-1)/8/L*(v[i]*tan(phi[i])-v[i+1]*tan(phi[i+1]));
minimize tracking:
 10000*(sum{i in 1..N} (if i=1 or i=N then 0.5 else 1)*((x[i]-ref[i,1])^2+(y[i]-ref[i,2])^2+(theta[i]-ref[i,3])^2)/(N-1)
 + time_weight*(tf/Tref-1)^2
 + smooth_weight/(N-1)*sum{i in 1..N-1}((a[i+1]-a[i])^2+(w[i+1]-w[i])^2));
subject to dx{i in 1..N-1}: x[i+1]-x[i]=tf/(N-1)/6*(v[i]*cos(theta[i])+4*vm[i]*cos(tm[i])+v[i+1]*cos(theta[i+1]));
subject to dy{i in 1..N-1}: y[i+1]-y[i]=tf/(N-1)/6*(v[i]*sin(theta[i])+4*vm[i]*sin(tm[i])+v[i+1]*sin(theta[i+1]));
subject to dtheta{i in 1..N-1}: theta[i+1]-theta[i]=tf/(N-1)/6/L*(v[i]*tan(phi[i])+4*vm[i]*tan(pm[i])+v[i+1]*tan(phi[i+1]));
subject to dv{i in 1..N-1}: v[i+1]-v[i]=tf/(N-1)/2*(a[i]+a[i+1]);
subject to dphi{i in 1..N-1}: phi[i+1]-phi[i]=tf/(N-1)/2*(w[i]+w[i+1]);
subject to vm_bound{i in 1..N-1}: -V<=vm[i]<=V;
subject to pm_bound{i in 1..N-1}: -P<=pm[i]<=P;
# Quadratic Bezier control points bound the ENTIRE v/phi interval, including
# an extremum that falls between a collocation node and its midpoint.
subject to v_bezier_bound{i in 1..N-1}: -V<=v[i]+tf/(N-1)/2*a[i]<=V;
subject to p_bezier_bound{i in 1..N-1}: -P<=phi[i]+tf/(N-1)/2*w[i]<=P;
subject to xstart: x[1]=q0[1];
subject to ystart: y[1]=q0[2];
subject to tstart: theta[1]=q0[3];
subject to vstart: v[1]=0;
subject to pstart: phi[1]=0;
subject to vfinish: v[N]=0;
subject to pfinish: phi[N]=0;
# Terminal pose intentionally unconstrained: its attainment is measured later.
# No obstacle positions, safety corridors or collision constraints appear here.
