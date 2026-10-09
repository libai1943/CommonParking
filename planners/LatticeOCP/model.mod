# Bergman et al. TIV 2021, complete car model and matched objective (14).
# Arc-length multiple shooting: RK4, constant steering-acceleration inputs.
param E integer>=1;param P integer>=1;param O integer>=0;param B integer>=1;
param phase{1..E} integer>=1,<=P;param num_steps{1..P} integer>=1;param gear{1..P};
param lw;param phimax;param rate_max;param acceleration_max;param gamma;param max_length;
param start{1..5};param goal{1..5};param terminal_mode integer>=0,<=2 default 0;
param body{1..B,1..3};param obstacle{1..O,1..3};param clearance;
var seg_length{1..P}>=0,<=max_length;
var z{0..E,1..5}; # x,y,theta,alpha,dalpha/ds
var u{1..E}>=-acceleration_max,<=acceleration_max;
var h{e in 1..E}=seg_length[phase[e]]/num_steps[phase[e]];
var amid{e in 1..E}=z[e-1,4]+.5*h[e]*z[e-1,5]+.125*h[e]^2*u[e];
var aend{e in 1..E}=z[e-1,4]+h[e]*z[e-1,5]+.5*h[e]^2*u[e];
var wmid{e in 1..E}=z[e-1,5]+.5*h[e]*u[e];
var tend1{e in 1..E}=gear[phase[e]]*tan(z[e-1,4])/lw;
var tend2{e in 1..E}=gear[phase[e]]*tan(amid[e])/lw;
var tend4{e in 1..E}=gear[phase[e]]*tan(aend[e])/lw;
minimize matched_cost:sum{e in 1..E}h[e]*(1+gamma*(u[e]^2+
 (z[e-1,4]^2+4*amid[e]^2+aend[e]^2+10*(z[e-1,5]^2+4*wmid[e]^2+z[e,5]^2))/6));
subject to dx{e in 1..E}:z[e,1]=z[e-1,1]+gear[phase[e]]*h[e]/6*(
 cos(z[e-1,3])+2*cos(z[e-1,3]+.5*h[e]*tend1[e])+2*cos(z[e-1,3]+.5*h[e]*tend2[e])+cos(z[e-1,3]+h[e]*tend2[e]));
subject to dy{e in 1..E}:z[e,2]=z[e-1,2]+gear[phase[e]]*h[e]/6*(
 sin(z[e-1,3])+2*sin(z[e-1,3]+.5*h[e]*tend1[e])+2*sin(z[e-1,3]+.5*h[e]*tend2[e])+sin(z[e-1,3]+h[e]*tend2[e]));
subject to dtheta{e in 1..E}:z[e,3]=z[e-1,3]+h[e]/6*(tend1[e]+4*tend2[e]+tend4[e]);
subject to dalpha{e in 1..E}:z[e,4]=aend[e];
subject to domega{e in 1..E}:z[e,5]=z[e-1,5]+h[e]*u[e];
subject to steering{i in 0..E}:-phimax<=z[i,4]<=phimax;
subject to steering_mid{e in 1..E}:-phimax<=amid[e]<=phimax;
subject to rate{i in 0..E}:-rate_max<=z[i,5]<=rate_max;
subject to initial{s in 1..5}:z[0,s]=start[s];
subject to terminal_internal{s in 3..5}:z[E,s]=goal[s];
subject to terminal_position{s in 1..2:terminal_mode==0}:z[E,s]=goal[s];
# Free XY for heading-change maneuvers; local lateral displacement for parallel.
subject to terminal_parallel{j in 1..1:terminal_mode==2}:z[E,2]=goal[2];
subject to avoid{i in 0..E,b in 1..B,o in 1..O}:
 (z[i,1]+body[b,1]*cos(z[i,3])-body[b,2]*sin(z[i,3])-obstacle[o,1])^2+
 (z[i,2]+body[b,1]*sin(z[i,3])+body[b,2]*cos(z[i,3])-obstacle[o,2])^2
 >=(body[b,3]+obstacle[o,3]+clearance)^2;
