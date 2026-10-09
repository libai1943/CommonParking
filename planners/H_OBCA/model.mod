# Paper Eq. (7) duals/objective, with the common five-state rear bicycle and RK2.
param N integer >=3;param M integer >=1;param E integer >=3;
param first{1..M} integer;param last{1..M} integer;
param A{1..E,1..2};param b{1..E};param g{1..4};
param lw;param vmax;param amax;param phimax;param wmax;param dmin;
param boundary{1..8};param tref;param scale_min;param scale_max;
param q_control{1..2};param q_rate{1..2};param time_weight;
var tf >= scale_min*tref, <= scale_max*tref;
var h=tf/(N-1);
var x{1..N};var y{1..N};var theta{1..N};var v{1..N} >= -vmax, <= vmax;
var phi{1..N} >= -phimax, <= phimax;var a{1..N-1} >= -amax, <= amax;
var omega{1..N-1} >=-wmax,<=wmax;
var lambda{1..N,1..E} >=0;var mu{1..N,1..M,1..4} >=0;
minimize cost: time_weight*tf+sum{i in 1..N-1}(q_control[1]*phi[i]^2+q_control[2]*a[i]^2)
 +sum{i in 2..N-1}(q_rate[1]*((phi[i]-phi[i-1])/h)^2+q_rate[2]*((a[i]-a[i-1])/h)^2)
 +q_rate[1]*(phi[1]/h)^2+q_rate[2]*(a[1]/h)^2;
subject to dx{i in 1..N-1}:x[i+1]=x[i]+h*(v[i]+h*a[i]/2)*cos(theta[i]+h*v[i]*tan(phi[i])/(2*lw));
subject to dy{i in 1..N-1}:y[i+1]=y[i]+h*(v[i]+h*a[i]/2)*sin(theta[i]+h*v[i]*tan(phi[i])/(2*lw));
subject to dt{i in 1..N-1}:theta[i+1]=theta[i]+h*(v[i]+h*a[i]/2)*tan(phi[i]+h*omega[i]/2)/lw;
subject to dv{i in 1..N-1}:v[i+1]=v[i]+h*a[i];
subject to steering_flow{i in 1..N-1}:phi[i+1]=phi[i]+h*omega[i];
subject to initial_steering:phi[1]=0;
subject to terminal_steering:phi[N]=0;
subject to start_x:x[1]=boundary[1];subject to start_y:y[1]=boundary[2];
subject to start_theta:theta[1]=boundary[3];subject to start_v:v[1]=boundary[4];
subject to goal_x:x[N]=boundary[5];subject to goal_y:y[N]=boundary[6];
subject to goal_theta:theta[N]=boundary[7];subject to goal_v:v[N]=boundary[8];
subject to dual_x{i in 1..N,j in 1..M}:
 mu[i,j,1]-mu[i,j,2]+cos(theta[i])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +sin(theta[i])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to dual_y{i in 1..N,j in 1..M}:
 mu[i,j,3]-mu[i,j,4]-sin(theta[i])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +cos(theta[i])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to normal_unit{i in 1..N,j in 1..M}:
 (sum{k in first[j]..last[j]}A[k,1]*lambda[i,k])^2+(sum{k in first[j]..last[j]}A[k,2]*lambda[i,k])^2=1;
subject to separation{i in 1..N,j in 1..M}:
 -sum{k in 1..4}g[k]*mu[i,j,k]+sum{k in first[j]..last[j]}(A[k,1]*x[i]+A[k,2]*y[i]-b[k])*lambda[i,k]>=dmin;
