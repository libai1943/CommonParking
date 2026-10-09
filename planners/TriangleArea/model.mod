# Adapted from the user-supplied Triangle21/NLP.mod. Multiple polygons share the same area constraints.
param BasicParameters{i in {1..19}};
param lw      := BasicParameters[1];
param lf      := BasicParameters[2];
param lr      := BasicParameters[3];
param lb      := BasicParameters[4];
param vmax    := BasicParameters[5];
param vmin    := BasicParameters[6];
param amax    := BasicParameters[7];
param amin    := BasicParameters[8];
param phimax  := BasicParameters[9];
param wmax    := BasicParameters[10];
param nfe     := BasicParameters[11];
param ngrids  := BasicParameters[12];
param x0      := BasicParameters[13];
param y0      := BasicParameters[14];
param theta0  := BasicParameters[15];
param xf      := BasicParameters[16];
param yf      := BasicParameters[17];
param thetaf  := BasicParameters[18];
param M integer >=1;
param first{1..M} integer;param last{1..M} integer;param following{1..ngrids} integer;
param area{1..M};param area_epsilon default 0.01;

var tf >= 0.1;
var hi = tf / (nfe - 1);
param Obstacle{i in {1..ngrids}, k in {1..2}};

var x{i in {1..nfe}};
var y{i in {1..nfe}};
var theta{i in {1..nfe}};
var v{i in {1..nfe}};
var a{i in {1..nfe}};
var phi{i in {1..nfe}};
var w{i in {1..nfe}};

minimize obj:
tf;
s.t. time_limit_common:
tf <= 100;

s.t. DIFF_dxdt {i in {2..nfe}}:
x[i] = x[i-1] + hi * v[i-1] * cos(theta[i-1]);
s.t. DIFF_dydt {i in {2..nfe}}:
y[i] = y[i-1] + hi * v[i-1] * sin(theta[i-1]);
s.t. DIFF_dthetadt {i in {2..nfe}}:
theta[i] = theta[i-1] + hi * tan(phi[i-1]) * v[i-1] / lw;
s.t. DIFF_dvdt {i in {2..nfe}}:
v[i] = v[i-1] + hi * a[i-1];
s.t. DIFF_dphidt {i in {2..nfe}}:
phi[i] = phi[i-1] + hi * w[i-1];

s.t. Bonds_phi {i in {1..nfe}}:
-phimax <= phi[i] <= phimax;
s.t. Bonds_v {i in {1..nfe}}:
vmin <= v[i] <= vmax;
s.t. Bonds_w {i in {1..nfe}}:
-wmax <= w[i] <= wmax;
s.t. Bonds_a {i in {1..nfe}}:
amin <= a[i] <= amax;

s.t. EQ_init_x :
x[1] = x0;
s.t. EQ_init_y :
y[1] = y0;
s.t. EQ_init_theta :
theta[1] = theta0;
s.t. EQ_init_v :
v[1] = 0;
s.t. EQ_init_phi :
phi[1] = 0;

s.t. EQ_end_x :
x[nfe] = xf;
s.t. EQ_end_y :
y[nfe] = yf;
s.t. EQ_end_theta :
theta[nfe] = thetaf;
s.t. EQ_end_v :
v[nfe] = 0;
s.t. EQ_end_phi :
phi[nfe] = 0;
s.t. EQ_end_w :
w[nfe] = 0;
s.t. EQ_end_a :
a[nfe] = 0;






var AX{i in {1..nfe}};
var AY{i in {1..nfe}};
var BX{i in {1..nfe}};
var BY{i in {1..nfe}};
var CX{i in {1..nfe}};
var CY{i in {1..nfe}};
var DX{i in {1..nfe}};
var DY{i in {1..nfe}};

s.t. RELATIONSHIP_AX {i in {1..nfe}}:
AX[i] = x[i] + (lf+lw) * cos(theta[i]) - 0.5*lb * sin(theta[i]);
s.t. RELATIONSHIP_AY {i in {1..nfe}}:
AY[i] = y[i] + (lf+lw) * sin(theta[i]) + 0.5*lb * cos(theta[i]);
s.t. RELATIONSHIP_BX {i in {1..nfe}}:
BX[i] = x[i] + (lf+lw) * cos(theta[i]) + 0.5*lb * sin(theta[i]);
s.t. RELATIONSHIP_BY {i in {1..nfe}}:
BY[i] = y[i] + (lf+lw) * sin(theta[i]) - 0.5*lb * cos(theta[i]);
s.t. RELATIONSHIP_CX {i in {1..nfe}}:
CX[i] = x[i] - lr * cos(theta[i]) + 0.5*lb * sin(theta[i]);
s.t. RELATIONSHIP_CY {i in {1..nfe}}:
CY[i] = y[i] - lr * sin(theta[i]) - 0.5*lb * cos(theta[i]);
s.t. RELATIONSHIP_DX {i in {1..nfe}}:
DX[i] = x[i] - lr * cos(theta[i]) - 0.5*lb * sin(theta[i]);
s.t. RELATIONSHIP_DY {i in {1..nfe}}:
DY[i] = y[i] - lr * sin(theta[i]) + 0.5*lb * cos(theta[i]);

s.t. EQ_A_OUTSIDE_OBSTACLE {i in {1..nfe}, m in {1..M}}:
sum {j in first[m]..last[m]} 0.5 * abs((Obstacle[j,1]-AX[i])*(Obstacle[following[j],2]-AY[i])-(Obstacle[j,2]-AY[i])*(Obstacle[following[j],1]-AX[i])) >= area[m] + area_epsilon;

s.t. EQ_B_OUTSIDE_OBSTACLE {i in {1..nfe}, m in {1..M}}:
sum {j in first[m]..last[m]} 0.5 * abs((Obstacle[j,1]-BX[i])*(Obstacle[following[j],2]-BY[i])-(Obstacle[j,2]-BY[i])*(Obstacle[following[j],1]-BX[i])) >= area[m] + area_epsilon;

s.t. EQ_C_OUTSIDE_OBSTACLE {i in {1..nfe}, m in {1..M}}:
sum {j in first[m]..last[m]} 0.5 * abs((Obstacle[j,1]-CX[i])*(Obstacle[following[j],2]-CY[i])-(Obstacle[j,2]-CY[i])*(Obstacle[following[j],1]-CX[i])) >= area[m] + area_epsilon;

s.t. EQ_D_OUTSIDE_OBSTACLE {i in {1..nfe}, m in {1..M}}:
sum {j in first[m]..last[m]} 0.5 * abs((Obstacle[j,1]-DX[i])*(Obstacle[following[j],2]-DY[i])-(Obstacle[j,2]-DY[i])*(Obstacle[following[j],1]-DX[i])) >= area[m] + area_epsilon;

s.t. EQ_OBSTACLE_OUTSIDE_ABCD {i in {1..nfe}, j in {1..ngrids}}:
0.5 * abs((AX[i]-Obstacle[j,1])*(BY[i]-Obstacle[j,2])-(AY[i]-Obstacle[j,2])*(BX[i]-Obstacle[j,1])) + 0.5 * abs((BX[i]-Obstacle[j,1])*(CY[i]-Obstacle[j,2])-(BY[i]-Obstacle[j,2])*(CX[i]-Obstacle[j,1])) + 0.5 * abs((CX[i]-Obstacle[j,1])*(DY[i]-Obstacle[j,2])-(CY[i]-Obstacle[j,2])*(DX[i]-Obstacle[j,1])) + 0.5 * abs((DX[i]-Obstacle[j,1])*(AY[i]-Obstacle[j,2])-(DY[i]-Obstacle[j,2])*(AX[i]-Obstacle[j,1])) >= (lf+lw+lr)*lb + area_epsilon;
