function initial=Seed(c,raw,N)
% A feasible continuous seed follows the same arcs, stopping for every
% curvature change so bounded steering rate does not require path deviation.
v=c.vehicle;p=raw.primitives(abs(raw.primitives(:,1))>1e-10,:);merged=zeros(0,2);
for j=1:size(p,1)
 if ~isempty(merged)&&sign(merged(end,1))==sign(p(j,1))&&abs(merged(end,2)-p(j,2))<1e-10
  merged(end,1)=merged(end,1)+p(j,1);
 else,merged(end+1,:)=p(j,:);end %#ok<AGROW>
end
phases={};pose=[c.task.x0 c.task.y0 c.task.theta0];phi=0;clock=0;
for j=1:size(merged,1)
 next=atan(v.lw*merged(j,2));duration=abs(next-phi)/v.wmax;
 if duration>1e-10
  phases{end+1}=struct('type','steer','start',clock,'duration',duration,'pose',pose,'from',phi,'to',next);clock=clock+duration; %#ok<AGROW>
 end
 phi=next;gear=sign(merged(j,1));speed=v.v_forward;if gear<0,speed=v.v_reverse;end
 profile=cpe.LongitudinalProfile(abs(merged(j,1)),speed,v.a_accel,v.a_brake);
 phases{end+1}=struct('type','move','start',clock,'duration',profile.tf,'pose',pose,'profile',profile,'gear',gear,'curvature',merged(j,2),'phi',phi); %#ok<AGROW>
 clock=clock+profile.tf;pose=parking.IntegratePrimitive(pose,merged(j,1),merged(j,2));
end
duration=abs(phi)/v.wmax;
if duration>1e-10,phases{end+1}=struct('type','steer','start',clock,'duration',duration,'pose',pose,'from',phi,'to',0);clock=clock+duration;end
t=linspace(0,clock,N)';q=zeros(N,7);
for j=1:numel(phases)
 s=phases{j};if j==numel(phases),mask=t>=s.start;else,mask=t>=s.start&t<s.start+s.duration;end
 local=t(mask)-s.start;
 if strcmp(s.type,'steer')
  q(mask,1:3)=repmat(s.pose,nnz(mask),1);q(mask,5)=s.from+(s.to-s.from)*local/s.duration;q(mask,7)=(s.to-s.from)/s.duration;
 else
  [distance,speed,accel]=cpe.LongitudinalAt(s.profile,local);pos=parking.IntegratePrimitive(s.pose,s.gear*distance,s.curvature);
  pos(:,3)=s.pose(3)+s.gear*distance*s.curvature;q(mask,:)=[pos,s.gear*speed,repmat(s.phi,nnz(mask),1),s.gear*accel,zeros(nnz(mask),1)];
 end
end
q(:,3)=unwrap(q(:,3));q([1 end],4:5)=0;
initial=struct('t',t,'x',q(:,1),'y',q(:,2),'theta',q(:,3),'v',q(:,4),'phi',q(:,5),'a',q(:,6),'omega',q(:,7));
end
