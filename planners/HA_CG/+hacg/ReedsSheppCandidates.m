function [paths,costs]=ReedsSheppCandidates(start,goal,kappa)
% Copyright (c) 2010, Rice University. Copyright (c) 2017, see licenses/NOTICE.
% SPDX-License-Identifier: Apache-2.0 AND BSD-3-Clause
% Numeric arc candidates from the Reeds--Shepp formula families in
% steering_functions (Banzhaf et al., Apache-2.0), derived from OMPL
% (Rice University, BSD-3-Clause). See accompanying third-party notices.
% Unit forward/reverse cost; each candidate is [signed_length, curvature].
% No obstacle checks or cached answers. MATLAB adaptation by CommonParking.
d=goal(1:2)-start(1:2);cs=cos(start(3));sn=sin(start(3));
x=kappa*(cs*d(1)+sn*d(2));y=kappa*(-sn*d(1)+cs*d(2));phi=wrap(goal(3)-start(3));
X=[x -x x -x];Y=[y y -y -y];P=[phi -phi -phi phi];
xb=x*cos(phi)+y*sin(phi);yb=x*sin(phi)-y*cos(phi);
XB=[X xb -xb xb -xb];YB=[Y yb yb -yb -yb];PB=[P P];
paths=cell(0,1);costs=zeros(0,1);zero=10*eps;
% CSC, formula 8.1 (LpSpLp).
u=hypot(X-sin(P),Y-1+cos(P));t=atan2(Y-1+cos(P),X-sin(P));v=wrap(P-t);
[paths,costs]=append(paths,costs,[t;u;v],[1 0 1],t>=-zero & v>=-zero,kappa);
% CSC, formula 8.2 (LpSpRp).
a=X+sin(P);b=Y-1-cos(P);r2=a.^2+b.^2;u=sqrt(max(0,r2-4));t=wrap(atan2(b,a)+atan2(2,u));v=wrap(t-P);
[paths,costs]=append(paths,costs,[t;u;v],[1 0 -1],r2>=4 & t>=-zero & v>=-zero,kappa);
% CCC, formula 8.3/8.4, including backwards configurations.
a=XB-sin(PB);b=YB-1+cos(PB);r=hypot(a,b);u=-2*asin(min(1,r/4));t=wrap(atan2(b,a)+u/2+pi);v=wrap(PB-t+u);
[paths,costs]=append(paths,costs,[t;u;v],[1 -1 1],r<=4 & t>=-zero & u<=zero,kappa);
% CCCC, formula 8.7.
a=X+sin(P);b=Y-1-cos(P);rho=(2+hypot(a,b))/4;u=acos(min(1,rho));[t,v]=tauOmega(u,-u,a,b,P);
[paths,costs]=append(paths,costs,[t;u;-u;v],[1 -1 1 -1],rho<=1 & t>=-zero & v<=zero,kappa);
% CCCC, formula 8.8.
rho=(20-a.^2-b.^2)/16;u=-acos(max(0,min(1,rho)));[t,v]=tauOmega(u,u,a,b,P);
[paths,costs]=append(paths,costs,[t;u;u;v],[1 -1 1 -1],rho>=0 & rho<=1 & u>=-pi/2 & t>=-zero & v>=-zero,kappa);
% CCSC, formula 8.9, including backwards configurations.
a=XB-sin(PB);b=YB-1+cos(PB);rho=hypot(a,b);r=sqrt(max(0,rho.^2-4));u=2-r;t=wrap(atan2(b,a)+atan2(r,-2));v=wrap(PB-pi/2-t);
[paths,costs]=append(paths,costs,[t;-pi/2*ones(size(t));u;v],[1 -1 0 1],rho>=2 & t>=-zero & u<=zero & v<=zero,kappa);
% CCSC, formula 8.10, including backwards configurations.
a=XB+sin(PB);b=YB-1-cos(PB);rho=hypot(a,b);t=atan2(a,-b);u=2-rho;v=wrap(t+pi/2-PB);
[paths,costs]=append(paths,costs,[t;-pi/2*ones(size(t));u;v],[1 -1 0 -1],rho>=2 & t>=-zero & u<=zero & v<=zero,kappa);
% CCSCC, formula 8.11.
a=X+sin(P);b=Y-1-cos(P);rho=hypot(a,b);u=4-sqrt(max(0,rho.^2-4));t=wrap(atan2((4-u).*a-2*b,-2*a+(u-4).*b));v=wrap(t-P);
[paths,costs]=append(paths,costs,[t;-pi/2*ones(size(t));u;-pi/2*ones(size(t));v],[1 -1 0 1 -1],rho>=2 & u<=zero & t>=-zero & v>=-zero,kappa);

end
function [paths,costs]=append(paths,costs,lengths,turns,valid,kappa)
for j=find(valid)
    timeSign=1-2*(mod(j,2)==0);reflection=1-2*(mod(floor((j-1)/2),2)==1);
    pp=[timeSign*lengths(:,j)/kappa,reflection*turns(:)*kappa];
    if j>4,pp=flipud(pp);end
    pp(abs(pp(:,1))<1e-10,:)=[];paths{end+1,1}=pp;costs(end+1,1)=sum(abs(pp(:,1))); %#ok<AGROW>
end
end
function [tau,omega]=tauOmega(u,v,xi,eta,phi)
delta=wrap(u-v);A=sin(u)-sin(delta);B=cos(u)-cos(delta)-1;
t1=atan2(eta.*A-xi.*B,xi.*A+eta.*B);t2=2*(cos(delta)-cos(v)-cos(u))+3;
tau=wrap(t1+pi*(t2<0));omega=wrap(tau-u+v-phi);
end
function angle=wrap(angle)
angle=rem(angle,2*pi);angle(angle<-pi)=angle(angle<-pi)+2*pi;angle(angle>pi)=angle(angle>pi)-2*pi;
end
