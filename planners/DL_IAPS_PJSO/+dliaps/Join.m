function q=Join(q,local,wmax)
% At rest, rotate the steering at the common rate limit before moving.
if isempty(q)
 q=local;delta=q.phi(1);dt=abs(delta)/wmax;
 if dt>1e-10
  fields=fieldnames(q);start=q;
  for j=1:numel(fields),f=fields{j};start.(f)=q.(f)(1);end
  start.t=0;start.phi=0;start.v=0;start.a=0;start.omega=delta/dt;q.t=q.t+dt;
  for j=1:numel(fields),f=fields{j};q.(f)=[start.(f);q.(f)];end
 end
elseif isempty(local)
 delta=-q.phi(end);dt=abs(delta)/wmax;
 if dt>1e-10
  fields=fieldnames(q);q.omega(end)=delta/dt;
  for j=1:numel(fields),f=fields{j};q.(f)(end+1)=q.(f)(end);end
  q.t(end)=q.t(end-1)+dt;q.phi(end)=0;q.v(end)=0;q.a(end)=0;q.omega(end)=0;
 end
else
 local.theta=local.theta+2*pi*round((q.theta(end)-local.theta(1))/(2*pi));
 assert(norm([q.x(end)-local.x(1),q.y(end)-local.y(1),q.theta(end)-local.theta(1)],Inf)<1e-6);
 delta=local.phi(1)-q.phi(end);dt=abs(delta)/wmax;fields=fieldnames(q);
 if dt>1e-10,q.omega(end)=delta/dt;keep=1:numel(local.t);else,keep=2:numel(local.t);end
 local.t=local.t+q.t(end)+dt;
 for j=1:numel(fields),f=fields{j};q.(f)=[q.(f);local.(f)(keep)];end
end
end
