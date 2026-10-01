# Shi et al. (2019), Eqs. (1), (7), (9) and Appendix (11)-(12).
# Global Legendre-Gauss-Lobatto pseudospectral transcription.
param N integer>=3;param M integer>=1;param NV{1..M} integer>=3;
param D{1..N,1..N};param tau{1..N};param weight{1..N};
param obstacle{m in 1..M,1..NV[m],1..2};param body{1..4,1..2};
param lw;param vmax;param phimax;param wmax;param boundary{1..6};
param epsilon>0;param delta>0;param max_time;
var tf>=.1,<=max_time;
var x{1..N};var y{1..N};var theta{1..N};var phi{1..N}>=-phimax,<=phimax;
var v{1..N}>=-vmax,<=vmax;var omega{1..N}>=-wmax,<=wmax;
var bx{i in 1..N,j in 1..4}=x[i]+body[j,1]*cos(theta[i])-body[j,2]*sin(theta[i]);
var body_y{i in 1..N,j in 1..4}=y[i]+body[j,1]*sin(theta[i])+body[j,2]*cos(theta[i]);
# p = (obstacle weights, body weights, z_x,z_y,z_mass_A,z_mass_B).
var p{i in 1..N,m in 1..M,1..NV[m]+8}>=0;
var lambda{i in 1..N,m in 1..M,1..NV[m]+8}>=0;
var nu{i in 1..N,m in 1..M,1..4};
var stationarity{i in 1..N,m in 1..M,j in 1..NV[m]+8} =
 (if j<=NV[m] then obstacle[m,j,1]*nu[i,m,1]+obstacle[m,j,2]*nu[i,m,2]+nu[i,m,3]
 else if j<=NV[m]+4 then -bx[i,j-NV[m]]*nu[i,m,1]-body_y[i,j-NV[m]]*nu[i,m,2]+nu[i,m,4]
 else 1+nu[i,m,j-NV[m]-4])-lambda[i,m,j];
minimize cost:tf+tf/2*sum{i in 1..N}weight[i]*v[i]^2;
subject to dx{i in 1..N}:sum{j in 1..N}D[i,j]*x[j]=tf/2*v[i]*cos(theta[i]);
subject to dy{i in 1..N}:sum{j in 1..N}D[i,j]*y[j]=tf/2*v[i]*sin(theta[i]);
subject to dt{i in 1..N}:sum{j in 1..N}D[i,j]*theta[j]=tf/2*v[i]*tan(phi[i])/lw;
subject to dp{i in 1..N}:sum{j in 1..N}D[i,j]*phi[j]=tf/2*omega[i];
subject to initial_x:x[1]=boundary[1];subject to initial_y:y[1]=boundary[2];subject to initial_theta:theta[1]=boundary[3];
subject to final_x:x[N]=boundary[4];subject to final_y:y[N]=boundary[5];subject to final_theta:theta[N]=boundary[6];
subject to initial_v:v[1]=0;subject to final_v:v[N]=0;subject to initial_phi:phi[1]=0;
subject to primal_x{i in 1..N,m in 1..M}:
 sum{j in 1..NV[m]}obstacle[m,j,1]*p[i,m,j]-sum{j in 1..4}bx[i,j]*p[i,m,NV[m]+j]+p[i,m,NV[m]+5]=0;
subject to primal_y{i in 1..N,m in 1..M}:
 sum{j in 1..NV[m]}obstacle[m,j,2]*p[i,m,j]-sum{j in 1..4}body_y[i,j]*p[i,m,NV[m]+j]+p[i,m,NV[m]+6]=0;
subject to mass_a{i in 1..N,m in 1..M}:sum{j in 1..NV[m]}p[i,m,j]+p[i,m,NV[m]+7]=1;
subject to mass_b{i in 1..N,m in 1..M}:sum{j in 1..4}p[i,m,NV[m]+j]+p[i,m,NV[m]+8]=1;
subject to safety{i in 1..N,m in 1..M}:sum{j in NV[m]+5..NV[m]+8}p[i,m,j]>=epsilon+delta;
subject to equilibrium{i in 1..N,m in 1..M}:sum{j in 1..NV[m]+8}stationarity[i,m,j]^2<=epsilon;
subject to complementarity{i in 1..N,m in 1..M}:sum{j in 1..NV[m]+8}lambda[i,m,j]*p[i,m,j]<=epsilon;
