# Li & Shao 2015, literal printed Eq. (1), front-axle reference point.
# Fully implicit three-stage Radau collocation, equations (7)-(12).
param NE integer>=1;param M integer>=1;param V integer>=3;
param D{1..3,0..3};param tau{0..3};
param first{1..M} integer;param last{1..M} integer;param following{1..V} integer;
param obstacle{1..V,1..2};param area{1..M};param body{1..4,1..2};
param lw;param total_length;param width;param vmax;param amax;param phimax;param wmax;
param boundary{1..6};param area_margin;param max_time;
var tf>=.1,<=max_time;
var z{1..NE,0..3,1..5}; # x_front, y_front, theta, v, phi
var u{1..NE,1..3,1..2}; # acceleration, steering rate
var vx{e in 1..NE,k in 1..3,j in 1..4}=z[e,k,1]+body[j,1]*cos(z[e,k,3])-body[j,2]*sin(z[e,k,3]);
var vy{e in 1..NE,k in 1..3,j in 1..4}=z[e,k,2]+body[j,1]*sin(z[e,k,3])+body[j,2]*cos(z[e,k,3]);
minimize time:tf;
subject to dx{e in 1..NE,k in 1..3}:sum{j in 0..3}D[k,j]*z[e,j,1]=tf/NE*z[e,k,4]*cos(z[e,k,3]);
subject to dy{e in 1..NE,k in 1..3}:sum{j in 0..3}D[k,j]*z[e,j,2]=tf/NE*z[e,k,4]*sin(z[e,k,3]);
subject to dt{e in 1..NE,k in 1..3}:sum{j in 0..3}D[k,j]*z[e,j,3]=tf/NE*z[e,k,4]*sin(z[e,k,5])/lw;
subject to dv{e in 1..NE,k in 1..3}:sum{j in 0..3}D[k,j]*z[e,j,4]=tf/NE*u[e,k,1];
subject to dphi{e in 1..NE,k in 1..3}:sum{j in 0..3}D[k,j]*z[e,j,5]=tf/NE*u[e,k,2];
subject to continuity{e in 1..NE-1,s in 1..5}:z[e+1,0,s]=z[e,3,s];
subject to speed{e in 1..NE,k in 0..3}:-vmax<=z[e,k,4]<=vmax;
subject to steering{e in 1..NE,k in 0..3}:-phimax<=z[e,k,5]<=phimax;
subject to acceleration{e in 1..NE,k in 1..3}:-amax<=u[e,k,1]<=amax;
subject to steering_rate{e in 1..NE,k in 1..3}:-wmax<=u[e,k,2]<=wmax;
subject to start_pose{s in 1..3}:z[1,0,s]=boundary[s];
subject to goal_pose{s in 1..3}:z[NE,3,s]=boundary[s+3];
subject to start_v:z[1,0,4]=0;subject to goal_v:z[NE,3,4]=0;
subject to start_phi:z[1,0,5]=0;
subject to corner_outside{e in 1..NE,k in 1..3,m in 1..M,c in 1..4}:
 sum{j in first[m]..last[m]}abs((obstacle[j,1]-vx[e,k,c])*(obstacle[following[j],2]-vy[e,k,c])
 -(obstacle[j,2]-vy[e,k,c])*(obstacle[following[j],1]-vx[e,k,c]))/2>=area[m]+area_margin;
subject to obstacle_vertex_outside{e in 1..NE,k in 1..3,j in 1..V}:
 sum{c in 1..4}abs((vx[e,k,c]-obstacle[j,1])*(vy[e,k,if c==4 then 1 else c+1]-obstacle[j,2])
 -(vy[e,k,c]-obstacle[j,2])*(vx[e,k,if c==4 then 1 else c+1]-obstacle[j,1]))/2>=total_length*width+area_margin;
