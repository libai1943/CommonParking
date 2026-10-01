function [metrics,collision] = Metrics(z,c,cfg)
[~,gap]=parking.FootprintClearance([z.x z.y z.theta],c,0);
collision=gap<=0; % Edge contact counts as collision. A frame counts only once.
error=[z.x(end)-c.task.xf,z.y(end)-c.task.yf, ...
    atan2(sin(z.theta(end)-c.task.thetaf),cos(z.theta(end)-c.task.thetaf))];
e=cfg.evaluation;
effort=trapz(z.t,z.a.^2+(z.v.*z.omega).^2);
steering=trapz(z.t,z.phi.^2);
motion=sign(z.v(abs(z.v)>=0.01));changes=sum(diff(motion)~=0);
metrics=struct('collision_percent',100*mean(collision), ...
    'terminal_reached',all(abs(error(1:2))<=e.terminal_xy_m)&&abs(error(3))<=e.terminal_heading_rad, ...
    'execution_time_s',z.t(end),'control_effort_integral',effort, ...
    'steering_integral',steering,'gear_changes',changes, ...
    'smoothness_cost',e.effort_weight*effort+e.steering_weight*steering+e.gear_change_weight*changes);
% The TPCAP term 100*tf is omitted: execution time is its own metric. No
% collision or terminal penalties are added to this smoothness component.
end
