function [q,info]=Export(b,env,o)
t=[0;cumsum(b.dt)];p=b.pose;
q=struct('t',t,'x',p(:,1),'y',p(:,2),'theta',p(:,3),'v',p(:,4),'phi',p(:,5), ...
'a',[diff(p(:,4))./b.dt;0],'omega',[diff(p(:,5))./b.dt;0]);
info=struct('success',all(isfinite(p),'all'),'code','native_five_state_band');
end
