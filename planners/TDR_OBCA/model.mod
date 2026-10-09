# TDR-OBCA Eqs. (2), (7)-(9). Time horizon is fixed by temporal warm start.
param N integer >=3;param M integer >=1;param E integer >=3;
param first{1..M} integer;param last{1..M} integer;
param A{1..E,1..2};param b{1..E};param g{1..4};
param lw;param vmax;param amax;param phimax;param wmax;param epsilon;
param h >0;param weights{1..6};param boundary{1..8};
param previous_phi{1..N-1} default 0;param previous_a{1..N-1} default 0;
var x{1..N};var y{1..N};var theta{1..N};var v{1..N} >= -vmax, <= vmax;
var phi{1..N-1} >= -phimax, <= phimax;var a{1..N-1} >= -amax, <= amax;
var lambda{2..N,1..E} >=0;var mu{2..N,1..M,1..4} >=0;var d{2..N,1..M} <= -epsilon;
minimize cost:
 sum{i in 2..N}(weights[1]*(x[i]^2+y[i]^2+v[i]^2+theta[i]^2)
 +weights[2]*((x[i]-x[i-1])^2+(y[i]-y[i-1])^2+(v[i]-v[i-1])^2+(theta[i]-theta[i-1])^2)
 +weights[3]*(phi[i-1]^2+a[i-1]^2)
 +weights[4]*((phi[i-1]-previous_phi[i-1])^2+(a[i-1]-previous_a[i-1])^2))
 +weights[5]*((x[N]-boundary[5])^2+(y[N]-boundary[6])^2+(theta[N]-boundary[7])^2+(v[N]-boundary[8])^2)
 +weights[6]*sum{i in 2..N,j in 1..M}d[i,j];
subject to dx{i in 1..N-1}: x[i+1]=x[i]+h*v[i]*cos(theta[i]);
subject to dy{i in 1..N-1}: y[i+1]=y[i]+h*v[i]*sin(theta[i]);
subject to dh{i in 1..N-1}: theta[i+1]=theta[i]+h*v[i]*tan(phi[i])/lw;
subject to dv{i in 1..N-1}: v[i+1]=v[i]+h*a[i];
subject to steering_rate{i in 2..N-1}: -wmax <= (phi[i]-phi[i-1])/h <= wmax;
subject to first_steering_rate: -wmax <= phi[1]/h <= wmax;
subject to start_x:x[1]=boundary[1];subject to start_y:y[1]=boundary[2];
subject to start_v:v[1]=boundary[4];subject to start_heading:theta[1]=boundary[3];
# Benchmark rest-to-rest requirement; terminal pose remains soft as in Eq. (9).
subject to terminal_rest:v[N]=0;
subject to dual_x{i in 2..N,j in 1..M}:
 mu[i,j,1]-mu[i,j,2]+cos(theta[i])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +sin(theta[i])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to dual_y{i in 2..N,j in 1..M}:
 mu[i,j,3]-mu[i,j,4]-sin(theta[i])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +cos(theta[i])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to normal_bound{i in 2..N,j in 1..M}:
 (sum{k in first[j]..last[j]}A[k,1]*lambda[i,k])^2+(sum{k in first[j]..last[j]}A[k,2]*lambda[i,k])^2<=1;
subject to separation{i in 2..N,j in 1..M}:
 -sum{k in 1..4}g[k]*mu[i,j,k]+sum{k in first[j]..last[j]}(A[k,1]*x[i]+A[k,2]*y[i]-b[k])*lambda[i,k]+d[i,j]=0;
