# Li et al. TITS 2022, Eq. (10), (17)-(22), Algorithm 3.
# First-order explicit Runge-Kutta; nonlinear equalities are PENALTIES.
param N integer >= 3;param D integer >= 2;
param lw;param vmax;param amax;param phimax;param wmax;param max_time;
param weight_energy;param weight_penalty;
param boundary{1..6};param offset{1..D,1..2};param box{1..N,1..D,1..4};
var tf >= .1, <= max_time;
var x{1..N};var y{1..N};var theta{1..N};
var v{1..N} >= -vmax, <= vmax;var phi{1..N} >= -phimax, <= phimax;
var a{1..N} >= -amax, <= amax;var w{1..N} >= -wmax, <= wmax;
var cx{i in 1..N,j in 1..D} >= box[i,j,1], <= box[i,j,2];
var cy{i in 1..N,j in 1..D} >= box[i,j,3], <= box[i,j,4];
var h=tf/(N-1);
var dynamic_penalty=h*sum{i in 1..N-1}(
 ((x[i+1]-x[i])/h-v[i]*cos(theta[i]))^2+
 ((y[i+1]-y[i])/h-v[i]*sin(theta[i]))^2+
 ((theta[i+1]-theta[i])/h-v[i]*tan(phi[i])/lw)^2+
 ((v[i+1]-v[i])/h-a[i])^2+((phi[i+1]-phi[i])/h-w[i])^2);
var geometry_penalty=h*sum{i in 2..N,j in 1..D}(
 (cx[i,j]-x[i]-offset[j,1]*cos(theta[i])+offset[j,2]*sin(theta[i]))^2+
 (cy[i,j]-y[i]-offset[j,1]*sin(theta[i])-offset[j,2]*cos(theta[i]))^2);
var heading_penalty=(sin(theta[N])-sin(boundary[6]))^2+(cos(theta[N])-cos(boundary[6]))^2;
var infeasibility=dynamic_penalty+geometry_penalty+heading_penalty;
var nominal_cost=tf+weight_energy*h*sum{i in 1..N-1}(a[i]^2+(v[i]*w[i])^2);
# Divide the WHOLE objective by weight_penalty; the minimizer is unchanged.
# Explicit scaling also keeps Ipopt's absolute dual/complementarity tests in
# the same numerical units as its relative convergence test.
minimize compound_cost: nominal_cost/weight_penalty+infeasibility;
subject to start_x:x[1]=boundary[1];subject to start_y:y[1]=boundary[2];
subject to start_theta:theta[1]=boundary[3];
subject to finish_x:x[N]=boundary[4];subject to finish_y:y[N]=boundary[5];
subject to start_v:v[1]=0;subject to finish_v:v[N]=0;
subject to start_phi:phi[1]=0;subject to finish_phi:phi[N]=0;
subject to start_a:a[1]=0;subject to finish_a:a[N]=0;
subject to start_w:w[1]=0;subject to finish_w:w[N]=0;
subject to start_cx{j in 1..D}:cx[1,j]=boundary[1]+offset[j,1]*cos(boundary[3])-offset[j,2]*sin(boundary[3]);
subject to start_cy{j in 1..D}:cy[1,j]=boundary[2]+offset[j,1]*sin(boundary[3])+offset[j,2]*cos(boundary[3]);
