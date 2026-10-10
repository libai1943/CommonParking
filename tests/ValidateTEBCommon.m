function check=ValidateTEBCommon(c,result)
% Independently reconstruct the five-state graph objective and exported clock.
v=c.vehicle;o=result.diagnostics.options;bands=result.diagnostics.candidates;
costs=zeros(numel(bands),1);
for j=1:numel(bands)
 b=bands(j);p=b.pose;h=b.dt;n=size(p,1);assert(size(p,2)==5&&all(h>o.minimumDt));
 dx=diff(p(:,1))-.5*h.*(p(1:end-1,4).*cos(p(1:end-1,3))+p(2:end,4).*cos(p(2:end,3)));
 dy=diff(p(:,2))-.5*h.*(p(1:end-1,4).*sin(p(1:end-1,3))+p(2:end,4).*sin(p(2:end,3)));
 dh=diff(p(:,3))-.5*h/v.lw.*(p(1:end-1,4).*tan(p(1:end-1,5))+p(2:end,4).*tan(p(2:end,5)));
 a=diff(p(:,4))./h;w=diff(p(:,5))./h;
 violations=[max(0,abs(p(:,4))-v.vmax);max(0,abs(p(:,5))-v.phimax);max(0,abs(a)-v.amax);max(0,abs(w)-v.wmax)];
 samples=[p(:,1:3);.5*(p(1:end-1,1:3)+p(2:end,1:3))];distance=independentDistance(samples,c);
 costs(j)=sum(h.^2)+o.dynamicsWeight*sum([dx;dy;dh].^2)+o.limitsWeight*sum(violations.^2)+o.obstacleWeight*sum(max(0,o.clearance-distance).^2,'all');
 assert(abs(costs(j)-b.cost)<1e-6*(1+b.cost));
 expected=[c.task.x0 c.task.y0 c.task.theta0 0 0;c.task.xf c.task.yf c.task.thetaf 0 0];
 error=p([1 end],:)-expected;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));assert(max(abs(error),[],'all')<1e-7);
end
[~,selected]=min(costs);assert(selected==result.diagnostics.selected);b=bands(selected);p=b.pose;h=b.dt;q=result.trajectory;
assert(isequal([q.x q.y q.theta q.v q.phi],p)&&max(abs(q.t-[0;cumsum(h)]))<1e-12);
assert(max(abs(q.a-[diff(p(:,4))./h;0]))<1e-12&&max(abs(q.omega-[diff(p(:,5))./h;0]))<1e-12);
for cycle=1:numel(result.diagnostics.cycles)
 stages=result.diagnostics.cycles{cycle}.stages;
 for j=1:numel(stages),s=stages{j};if isempty(s),continue;end
 assert(s.cost<=s.initial_cost+1e-8*(1+s.initial_cost));
 if s.success,assert(s.accepted_steps>0||ismember(s.code,{'stationary','small_step'}));end
 end
end
t=unique([(0:.001:q.t(end))';q.t(end)]);pose=interp1(q.t,p(:,1:3),t);[~,gap]=parking.FootprintClearance(pose,c,0);
f=[p(:,4).*cos(p(:,3)),p(:,4).*sin(p(:,3)),p(:,4).*tan(p(:,5))/v.lw];defect=diff(p(:,1:3))-.5*h.*(f(1:end-1,:)+f(2:end,:));
check=struct('passed',true,'states',size(p,1),'objective',costs(selected),'selected_cost_error',abs(costs(selected)-b.cost), ...
'native_max_dynamics_defect',max(abs(defect),[],'all'),'native_max_speed',max(abs(p(:,4))),'native_max_steering',max(abs(p(:,5))), ...
'native_max_acceleration',max(abs(q.a)),'native_max_steering_rate',max(abs(q.omega)), ...
'native_linear_reference_collision_percent',100*mean(gap<=0),'native_reference_time',q.t(end));
end
function out=independentDistance(q,c)
% Scalar SAT and all edge/vertex distances, independently of vectorized TEB code.
v=c.vehicle;body=[-v.lr,-v.lb/2;v.lw+v.lf,-v.lb/2;v.lw+v.lf,v.lb/2;-v.lr,v.lb/2];out=zeros(size(q,1),c.obstacle.num_obs);
for i=1:size(q,1)
 R=[cos(q(i,3)) -sin(q(i,3));sin(q(i,3)) cos(q(i,3))];B=body*R'+q(i,1:2);
 for m=1:c.obstacle.num_obs
 obs=c.obstacle.obs{m};P=[obs.x(:) obs.y(:)];if norm(P(1,:)-P(end,:))<1e-10,P(end,:)=[];end
 E=[diff([B;B(1,:)]);diff([P;P(1,:)])];axes=[-E(:,2),E(:,1)]./vecnorm(E,2,2);gap=-inf;
 for a=1:size(axes,1),pb=B*axes(a,:)';pp=P*axes(a,:)';gap=max(gap,max(min(pb)-max(pp),min(pp)-max(pb)));end
 if gap<=0,out(i,m)=gap;continue;end
 d=inf;
 for pair=1:2
 if pair==1,A=B;C=P;else,A=P;C=B;end
 for a=1:size(A,1),for k=1:size(C,1),edge=C(mod(k,size(C,1))+1,:)-C(k,:);s=max(0,min(1,dot(A(a,:)-C(k,:),edge)/dot(edge,edge)));d=min(d,norm(A(a,:)-C(k,:)-s*edge));end,end
 end
 out(i,m)=d;
 end
end
end
