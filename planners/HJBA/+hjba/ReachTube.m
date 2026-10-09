function [value,report]=ReachTube(target,grid,speed,omegaMax,horizon,cfl,forceMatlab)
% HJ reachability in increasing look-back time: min_u grad(V)' f.
% Lax-Friedrichs dissipation, ENO2 space, SSP-RK3 time, min with target.
if nargin<7,forceMatlab=false;end
h=[grid.x(2)-grid.x(1),grid.y(2)-grid.y(1),2*pi/numel(grid.theta)];
vx=reshape(speed*cos(grid.theta),1,1,[]);vy=reshape(speed*sin(grid.theta),1,1,[]);
rate=max(abs(vx)/h(1)+abs(vy)/h(2)+omegaMax/h(3),[],'all');
steps=max(1,ceil(horizon*rate/cfl));dt=horizon/steps;value=target;
accelerated=~forceMatlab&&exist('hjba_hj_mex','file')==3;
if accelerated
 value=hjba_hj_mex(target,h(1:2),grid.theta,speed,omegaMax,horizon,cfl);
else
for iteration=1:steps
 a=min(target,value+dt*rhs(value));b=min(target,.75*value+.25*(a+dt*rhs(a)));
 value=min(target,value/3+2*(b+dt*rhs(b))/3);
end
end
report=struct('steps',steps,'dt',dt,'lookback_s',horizon,'speed',speed,'omega_max',omegaMax,'minimum',min(value,[],'all'),'reachable_nodes',nnz(value<=0),'compiled_stencil',accelerated);
 function r=rhs(z)
  [xm,xp]=hjba.ENO2(z,h(1),1,false);[ym,yp]=hjba.ENO2(z,h(2),2,false);[tm,tp]=hjba.ENO2(z,h(3),3,true);
  r=vx.*(xm+xp)/2+vy.*(ym+yp)/2-omegaMax*abs((tm+tp)/2) ...
    +(abs(vx).*(xp-xm)+abs(vy).*(yp-ym)+omegaMax*(tp-tm))/2;
 end
end
