function q=Trajectory(p,speed,v,o)
% Evaluate the exact piecewise-constant-jerk speed and sampled spatial path.
t=speed.t(1);
for k=1:numel(speed.t)-1
 nodes=linspace(speed.t(k),speed.t(k+1),max(1,ceil((speed.t(k+1)-speed.t(k))/o.outputTimeStep))+1)';
 t=[t;nodes(2:end)]; %#ok<AGROW>
end
index=discretize(t,[-Inf;speed.t(2:end-1);Inf]);dt=t-speed.t(index);j=speed.jerk(index);
s=speed.s(index)+speed.v(index).*dt+.5*speed.a(index).*dt.^2+j.*dt.^3/6;
velocity=speed.v(index)+speed.a(index).*dt+.5*j.*dt.^2;acceleration=speed.a(index)+j.*dt;
assert(min(s)>-1e-6&&max(s)<p.s(end)+1e-6,'Invalid integrated speed position.');
s=max(0,min(p.s(end),s));s([1 end])=[0;p.s(end)];velocity([1 end])=0;acceleration([1 end])=0;
phi=atan(v.lw*p.kappa);spatialIndex=discretize(s,[-Inf;p.s(2:end-1);Inf]);slope=diff(phi)./diff(p.s);
q=struct('t',t,'x',interp1(p.s,p.x,s),'y',interp1(p.s,p.y,s),'theta',interp1(p.s,p.theta,s), ...
 'v',p.gear*velocity,'phi',interp1(p.s,phi,s),'a',p.gear*acceleration,'omega',slope(spatialIndex).*velocity);
end
