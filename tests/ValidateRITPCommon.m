function report=ValidateRITPCommon(c,result)
phases=result.diagnostics.phases;o=result.diagnostics.options;v=c.vehicle;
checks=cell(numel(phases),1);elapsed=abs(phases{1}.trajectory.phi(1))/v.wmax;full=result.trajectory;
for k=1:numel(phases)
 p=phases{k};assert(p.success&&p.solver.exitflag>0&&p.velocity.exitflag>0);
 C=p.coefficients;L=p.problem.L;ref=p.reference;S=[0;cumsum(vecnorm(diff(ref(:,1:2)),2,2))];
 [xy,d,dd,~]=values(C,p.problem.s,L);[refxy,~,~,~]=values(C,S,L);weights=ones(size(S));
 for j=1:p.iterations-1
  h=p.history{j};assert(h.collision_sample>0);ids=max(1,h.reference_index-j):min(numel(S),h.reference_index+j);weights(ids)=o.beta*weights(ids);
 end
 assert(max(abs(weights-p.problem.weights))<1e-8);
 objective=o.pathWeights(1)*sum(weights.*sum((refxy-ref(:,1:2)).^2,2))+o.pathWeights(2)*sum(d.^2,'all')+o.pathWeights(3)*sum(dd.^2,'all');
 z=C(:);qpObjective=.5*z'*p.problem.H*z+p.problem.f'*z+o.pathWeights(1)*sum(weights.*sum(ref(:,1:2).^2,2));
 assert(abs(objective-qpObjective)<1e-6*max(1,objective));
 eq=max(abs(p.problem.Aeq*z-p.problem.beq));assert(eq<1e-7);
 nullspace=null(p.problem.Aeq);stationarity=norm(nullspace'*(p.problem.H*z+p.problem.f),inf)/max(1,norm(p.problem.f,inf));assert(stationarity<1e-6);
 [~,tangent,~,~]=values(C,[0;L],L);expected=p.gear*[cos(ref([1 end],3)),sin(ref([1 end],3))];assert(max(abs(tangent-expected),[],'all')<1e-7);
 q=p.trajectory;T=q.t(end);assert(abs(T-p.velocity.T*p.time_scale)<1e-9);
 [independent,minimumTangent]=profile(C,L,p.gear,v.lw,q.t,T);observed=[q.x q.y q.theta q.v q.phi q.a q.omega];
 difference=observed-independent;difference(:,3)=atan2(sin(difference(:,3)),cos(difference(:,3)));mapping=max(abs(difference),[],'all');assert(mapping<1e-6);
 t=unique([(0:.001:T)';T]);dense=profile(C,L,p.gear,v.lw,t,T);
 bound=max([0;abs(dense(:,4))-v.vmax;abs(dense(:,5))-v.phimax;abs(dense(:,6))-v.amax;abs(dense(:,7))-v.wmax]);assert(bound<2e-6);
 [~,gap]=parking.FootprintClearance(dense(:,1:3),c,0);
 fields={'x','y','theta','v','phi','a'};
 for j=1:numel(fields)
  f=fields{j};actual=interp1(full.t,full.(f),elapsed+q.t);error=actual-q.(f);
  if strcmp(f,'theta'),error=atan2(sin(error),cos(error));end
  assert(max(abs(error))<1e-6);
 end
 elapsed=elapsed+T;
 if k<numel(phases),elapsed=elapsed+abs(phases{k+1}.trajectory.phi(1)-q.phi(end))/v.wmax;end
 checks{k}=struct('objective_error',abs(objective-qpObjective),'equality_residual',eq,'projected_gradient',stationarity, ...
  'output_chain_rule_error',mapping,'dense_common_limit_excess',bound,'minimum_sampled_tangent',minimumTangent, ...
  'dense_curve_collision_percent',100*mean(gap<=0),'duration_s',T,'time_scale',p.time_scale);
end
elapsed=elapsed+abs(phases{end}.trajectory.phi(end))/v.wmax;
assert(abs(full.t(end)-elapsed)<1e-7&&all(diff(full.t)>0));
assert(max(abs([full.v([1 end]);full.phi([1 end])]))<1e-7);
assert(max(abs(diff(full.phi)./diff(full.t)))<v.wmax+2e-6);
task=c.task;endpoint=[full.x(1)-task.x0;full.y(1)-task.y0;full.x(end)-task.xf;full.y(end)-task.yf;atan2(sin(full.theta([1 end])-[task.theta0;task.thetaf]),cos(full.theta([1 end])-[task.theta0;task.thetaf]))];
assert(max(abs(endpoint))<1e-7);
report=struct('passed',true,'phases',{checks},'endpoint_error',max(abs(endpoint)),'duration_s',full.t(end));
end
function [q,minTangent]=profile(C,L,gear,lw,t,T)
u=t/T;s=L*(10*u.^3-15*u.^4+6*u.^5);ds=L/T*(30*u.^2-60*u.^3+30*u.^4);dds=L/T^2*(60*u-180*u.^2+120*u.^3);
[xy,d,dd,ddd]=values(C,s,L);magnitude=vecnorm(d,2,2);cross=d(:,1).*dd(:,2)-d(:,2).*dd(:,1);dot=sum(d.*dd,2);
k=gear*cross./magnitude.^3;dk=gear*((d(:,1).*ddd(:,2)-d(:,2).*ddd(:,1))./magnitude.^3-3*cross.*dot./magnitude.^5);
theta=unwrap(atan2(gear*d(:,2),gear*d(:,1)));phi=atan(lw*k);omega=lw*dk./(1+(lw*k).^2).*ds;
q=[xy theta gear*magnitude.*ds phi gear*(magnitude.*dds+dot./magnitude.*ds.^2) omega];minTangent=min(magnitude);
end
function [p,d,dd,ddd]=values(C,s,L)
u=s/L;p=zeros(numel(s),2);d=p;dd=p;ddd=p;
for j=1:2,a=flipud(C(:,j));p(:,j)=polyval(a,u);d(:,j)=polyval(polyder(a),u)/L;dd(:,j)=polyval(polyder(polyder(a)),u)/L^2;ddd(:,j)=polyval(polyder(polyder(polyder(a))),u)/L^3;end
end
