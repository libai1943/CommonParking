# Convex distance-dual warm start, as in authors' DualMultWS.jl.
param N integer >=3;param M integer >=1;param E integer >=3;
param first{1..M} integer;param last{1..M} integer;
param A{1..E,1..2};param b{1..E};param g{1..4};param pose{1..N,1..3};
var lambda{1..N,1..E} >=0;var mu{1..N,1..M,1..4} >=0;
maximize clearance:sum{i in 1..N,j in 1..M}(
 -sum{k in 1..4}g[k]*mu[i,j,k]+sum{k in first[j]..last[j]}(A[k,1]*pose[i,1]+A[k,2]*pose[i,2]-b[k])*lambda[i,k]);
subject to dual_x{i in 1..N,j in 1..M}:
 mu[i,j,1]-mu[i,j,2]+cos(pose[i,3])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +sin(pose[i,3])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to dual_y{i in 1..N,j in 1..M}:
 mu[i,j,3]-mu[i,j,4]-sin(pose[i,3])*sum{k in first[j]..last[j]}A[k,1]*lambda[i,k]
 +cos(pose[i,3])*sum{k in first[j]..last[j]}A[k,2]*lambda[i,k]=0;
subject to normal_bound{i in 1..N,j in 1..M}:
 (sum{k in first[j]..last[j]}A[k,1]*lambda[i,k])^2+(sum{k in first[j]..last[j]}A[k,2]*lambda[i,k])^2<=1;
