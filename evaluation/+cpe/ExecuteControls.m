function [z,diagnostics] = ExecuteControls(native,c,cfg)
% Independently replay the optimized, piecewise-linear controls by RK4.
% Split at every control knot AND every evaluation frame; max step is 1 ms.
tf=native.t(end);frames=(0:cfg.evaluation.frame_dt_s:tf)';
if tf-frames(end)>1e-12,frames(end+1)=tf;else,frames(end)=tf;end
timeline=unique([frames;native.t]);n=numel(timeline);
states=zeros(n,5);states(1,:)=[c.task.x0 c.task.y0 c.task.theta0 0 0];
edge=1;L=c.vehicle.lw;
for j=1:n-1
    t=timeline(j);dt=timeline(j+1)-t;
    while edge<numel(native.t)-1&&t>=native.t(edge+1)-1e-13,edge=edge+1;end
    h=native.t(edge+1)-native.t(edge);
    alpha=(t-native.t(edge))/h;
    u0=[native.a(edge) native.omega(edge)];
    slope=([native.a(edge+1) native.omega(edge+1)]-u0)/h;
    u=u0+alpha*h*slope;
    q=states(j,:);k1=dynamics(q,u,L);k2=dynamics(q+dt*k1/2,u+dt*slope/2,L);
    k3=dynamics(q+dt*k2/2,u+dt*slope/2,L);k4=dynamics(q+dt*k3,u+dt*slope,L);
    states(j+1,:)=q+dt*(k1+2*k2+2*k3+k4)/6;
end
[~,ids]=ismember(native.t,timeline);
delta=states(ids,:)-[native.x native.y native.theta native.v native.phi];
diagnostics=struct('max_pose_defect',max(abs(delta(:,1:3)),[],'all'), ...
    'max_state_defect',max(abs(delta),[],'all'),'integrator','RK4, <=1 ms, split at control knots');
% With linear controls, v and phi are quadratic on every interval. Check
% their interior extrema analytically rather than trusting sampled bounds.
maxV=max(abs(states(:,4)));maxP=max(abs(states(:,5)));
for j=1:numel(native.t)-1
    h=native.t(j+1)-native.t(j);
    for field=1:2
        if field==1,u=native.a;column=4;else,u=native.omega;column=5;end
        slope=(u(j+1)-u(j))/h;
        if slope==0,continue;end
        tau=-u(j)/slope;
        if tau>0&&tau<h
            value=states(ids(j),column)+u(j)*tau+.5*slope*tau^2;
            if field==1,maxV=max(maxV,abs(value));else,maxP=max(maxP,abs(value));end
        end
    end
end
factor=1+cfg.evaluation.limit_relaxation;
diagnostics.max_dynamic_violation=max([0,maxV-c.vehicle.vmax*factor, ...
    maxP-c.vehicle.phimax*factor,max(abs(native.a))-c.vehicle.amax*factor, ...
    max(abs(native.omega))-c.vehicle.wmax*factor]);
diagnostics.max_speed=maxV;diagnostics.max_steering=maxP;
[~,ids]=ismember(frames,timeline);q=states(ids,:);
z=struct('t',frames,'x',q(:,1),'y',q(:,2),'theta',q(:,3),'v',q(:,4),'phi',q(:,5), ...
    'a',interp1(native.t,native.a,frames,'linear'), ...
    'omega',interp1(native.t,native.omega,frames,'linear'));
end
function derivative=dynamics(q,u,L)
derivative=[q(4)*cos(q(3)),q(4)*sin(q(3)),q(4)*tan(q(5))/L,u];
end
