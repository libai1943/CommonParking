# Sections III-B/D/E: uniform-grid Crank-Nicolson, SO(2) differences,
# quasi-time-optimal cost and control-difference limits.
param N integer >= 2;param M integer >=0;param E integer >=0;
param first{1..M} integer;param last{1..M} integer;
param A{1..E,1..2};param b{1..E};param g{1..4};
param radius;param lw;param vmax;param amax;param phimax;param wmax;param dmin;
param min_dt;param speed_cost;param boundary{1..6};
var h >= min_dt;
var x{0..N};var y{0..N};var theta{0..N};
var v{0..N} >=-vmax,<=vmax;var phi{0..N} >=-phimax,<=phimax;
var lambda{0..N-1,1..E} >=0;var mu{0..N-1,1..M,1..2,1..4} >=0;
minimize cost: h*sum{i in 0..N-1}(1+.5*speed_cost*(v[i]^2+v[i+1]^2));
subject to dx{i in 0..N-1}:(x[i+1]-x[i])/h=.5*(v[i]*cos(theta[i])+v[i+1]*cos(theta[i+1]));
subject to dy{i in 0..N-1}:(y[i+1]-y[i])/h=.5*(v[i]*sin(theta[i])+v[i+1]*sin(theta[i+1]));
subject to dtheta{i in 0..N-1}:atan2(sin(theta[i+1]-theta[i]),cos(theta[i+1]-theta[i]))/h= .5*(v[i]*tan(phi[i])+v[i+1]*tan(phi[i+1]))/lw;
subject to accel{i in 0..N-1}:-amax<=(v[i+1]-v[i])/h<=amax;
subject to steer_rate{i in 0..N-1}:-wmax<=(phi[i+1]-phi[i])/h<=wmax;
subject to initial_rest:v[0]=0;
subject to final_rest:v[N]=0;
subject to initial_steering:phi[0]=0;
subject to final_steering:phi[N]=0;
subject to start_x:x[0]=boundary[1];subject to start_y:y[0]=boundary[2];
subject to start_theta:atan2(sin(theta[0]-boundary[3]),cos(theta[0]-boundary[3]))=0;
subject to goal_x:x[N]=boundary[4];subject to goal_y:y[N]=boundary[5];
subject to goal_theta:atan2(sin(theta[N]-boundary[6]),cos(theta[N]-boundary[6]))=0;
# One world-space separating normal covers both endpoint rectangles.
# Rotation remainder bounds deviation of each corner from its linear chord.
var rotation_bound{i in 0..N-1}=radius/8*atan2(sin(theta[i+1]-theta[i]),cos(theta[i+1]-theta[i]))^2;
subject to dual_x{i in 0..N-1,j in 1..M,s in 1..2}:
 mu[i,j,s,1]-mu[i,j,s,2]+cos(theta[i+s-1])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +sin(theta[i+s-1])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to dual_y{i in 0..N-1,j in 1..M,s in 1..2}:
 mu[i,j,s,3]-mu[i,j,s,4]-sin(theta[i+s-1])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +cos(theta[i+s-1])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to norm_bound{i in 0..N-1,j in 1..M}:
 (sum{k in first[j]..last[j]}A[k,1]*lambda[i,k])^2+(sum{k in first[j]..last[j]}A[k,2]*lambda[i,k])^2<=1;
subject to separation{i in 0..N-1,j in 1..M,s in 1..2}:
 -sum{k in 1..4}g[k]*mu[i,j,s,k]+sum{k in first[j]..last[j]}(A[k,1]*x[i+s-1]+A[k,2]*y[i+s-1]-b[k])*lambda[i,k]>=dmin+rotation_bound[i];
