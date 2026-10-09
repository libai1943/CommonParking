function [local,points,frame]=Frame(c,o)
% Rigid coordinates only: neither obstacles nor endpoint poses are changed.
angle=c.task.theta0;R=[cos(angle),sin(angle);-sin(angle),cos(angle)];origin=[c.task.x0,c.task.y0];
points=(ppocp.Points(c,o.pointTarget)-origin)*R';goal=R*([c.task.xf;c.task.yf]-origin');local=c;
local.task.x0=0;local.task.y0=0;local.task.theta0=0;local.task.xf=goal(1);local.task.yf=goal(2);
local.task.thetaf=atan2(sin(c.task.thetaf-angle),cos(c.task.thetaf-angle));
frame=struct('R',R,'origin',origin,'angle',angle);
end
