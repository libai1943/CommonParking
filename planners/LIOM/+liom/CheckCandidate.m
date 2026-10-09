function check = CheckCandidate(c,q,boxes,discs,options)
% Verify an inexact inner iterate before using it in the next outer problem.
% Large nonlinear residuals are allowed for continuation, never for success.
check=struct('valid',false,'infeasibility',inf,'bound_violation',inf, ...
    'endpoint_error',inf,'message','Missing or invalid candidate.');
N=options.nodes;D=discs.count;
fields={'t','x','y','theta','v','phi','a','omega'};
if ~isstruct(q)||~isscalar(q),return;end
for k=1:numel(fields)
    if ~isfield(q,fields{k}),return;end
    z=q.(fields{k});
    if ~isnumeric(z)||~isreal(z)||~isequal(size(z),[N 1])||any(~isfinite(z)),return;end
end
for key={'cx','cy'}
    if ~isfield(q,key{1}),return;end
    z=q.(key{1});
    if ~isnumeric(z)||~isreal(z)||~isequal(size(z),[N D])||any(~isfinite(z),'all'),return;end
end
if ~isequal(size(boxes),[N D 4])||~isreal(boxes)||any(~isfinite(boxes),'all'),return;end
T=q.t(end);h=T/(N-1);v=c.vehicle;t=c.task;o=discs.offsets;
if T<.1-1e-9||T>options.maxTime+1e-9||any(diff(q.t)<=0) ...
        ||max(abs(q.t-linspace(0,T,N)'))>1e-9,return;end
expectedX=q.x+cos(q.theta)*o(:,1)'-sin(q.theta)*o(:,2)';
expectedY=q.y+sin(q.theta)*o(:,1)'+cos(q.theta)*o(:,2)';
check.bound_violation=max([0;abs(q.v)-v.vmax;abs(q.phi)-v.phimax; ...
    abs(q.a)-v.amax;abs(q.omega)-v.wmax;reshape(boxes(:,:,1)-q.cx,[],1); ...
    reshape(q.cx-boxes(:,:,2),[],1);reshape(boxes(:,:,3)-q.cy,[],1); ...
    reshape(q.cy-boxes(:,:,4),[],1)]);
boundary=[q.x(1)-t.x0;q.y(1)-t.y0;q.theta(1)-t.theta0; ...
    q.x(end)-t.xf;q.y(end)-t.yf;q.v([1 end]);q.phi([1 end]); ...
    q.a([1 end]);q.omega([1 end]);(q.cx(1,:)-expectedX(1,:))';(q.cy(1,:)-expectedY(1,:))'];
check.endpoint_error=max(abs(boundary));
state=[q.x q.y q.theta q.v q.phi];
rhs=[q.v.*cos(q.theta) q.v.*sin(q.theta) q.v.*tan(q.phi)/v.lw q.a q.omega];
defect=diff(state)/h-rhs(1:end-1,:);
check.dynamic_penalty=h*sum(defect.^2,'all');
check.geometry_penalty=h*sum((q.cx(2:end,:)-expectedX(2:end,:)).^2+ ...
    (q.cy(2:end,:)-expectedY(2:end,:)).^2,'all');
check.heading_penalty=(sin(q.theta(end))-sin(t.thetaf))^2+(cos(q.theta(end))-cos(t.thetaf))^2;
check.infeasibility=check.dynamic_penalty+check.geometry_penalty+check.heading_penalty;
check.nominal_cost=T+options.energyWeight*h*sum(q.a(1:end-1).^2+(q.v(1:end-1).*q.omega(1:end-1)).^2);
check.valid=check.bound_violation<=1e-6&&check.endpoint_error<=1e-6&&isfinite(check.infeasibility);
check.message='Checked finite iterate, hard box/control bounds and linear endpoints.';
end
