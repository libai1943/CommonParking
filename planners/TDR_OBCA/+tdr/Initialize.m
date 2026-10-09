function [q,info] = Initialize(c,primitives,o)
gear=sign(primitives(:,1));cuts=[1;find(diff(gear)~=0)+1;size(primitives,1)+1];
ends=[0;cumsum(abs(primitives(:,1)))];profiles=cell(numel(cuts)-1,1);checks=profiles;times=0;
for j=1:numel(profiles)
    [profiles{j},checks{j}]=tdr.Speed(ends(cuts(j+1))-ends(cuts(j)),c.vehicle,o);
    if ~checks{j}.success,q=[];info=struct('success',false,'phases',{checks});return;end
    times(j+1,1)=times(j)+profiles{j}.t(end);
end
t=linspace(0,times(end),o.nodes)';s=zeros(size(t));v=s;a=s;
for i=1:numel(t)
    j=find(t(i)>=times(1:end-1),1,'last');z=profiles{j};
    elapsed=t(i)-times(j);k=min(numel(z.t)-1,find(elapsed>=z.t,1,'last'));dt=elapsed-z.t(k);
    s(i)=ends(cuts(j))+z.s(k)+z.v(k)*dt+.5*z.a(k)*dt^2+z.jerk(k)*dt^3/6;
    v(i)=gear(cuts(j))*(z.v(k)+z.a(k)*dt+.5*z.jerk(k)*dt^2);
    a(i)=gear(cuts(j))*(z.a(k)+z.jerk(k)*dt);
end
s=max(0,min(ends(end),s));
[pose,kappa]=cp.ArcPose([c.task.x0,c.task.y0,c.task.theta0],primitives,s);
q=struct('t',t,'x',pose(:,1),'y',pose(:,2),'theta',pose(:,3),'v',v, ...
    'a',a,'phi',atan(c.vehicle.lw*kappa));q.omega=[diff(q.phi)./diff(t);0];
info=struct('success',true,'phases',{checks},'profiles',{profiles},'phase_times',times,'mileage',s);
end
