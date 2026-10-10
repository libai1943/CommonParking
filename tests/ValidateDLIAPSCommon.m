function check=ValidateDLIAPSCommon(c,result)
o=result.diagnostics.options;v=c.vehicle;q=result.trajectory;
maxG=-Inf;maxDynamics=0;maxBounds=0;maxMapping=0;minGap=Inf;clearances=[];
first=result.diagnostics.maneuvers{1}.path;elapsed=abs(atan(v.lw*first.kappa(1)))/v.wmax;cusps=[];
for run=1:numel(result.diagnostics.maneuvers)
 m=result.diagnostics.maneuvers{run};p=m.path;last=m.smoothing.iterations{end};
 assert(m.smoothing.success&&last.success&&last.exitflag>0&&m.speed_solver.success&&m.speed_solver.exitflag>0);
 points=[p.x p.y];bending=2*points(2:end-1,:)-points(1:end-2,:)-points(3:end,:);chords=diff(points(1:end-1,:));
 g=(sum(bending.^2,2)-last.curvature_limit^2*sum(chords.^2,2).^2)/last.constraint_scale;maxG=max(maxG,max(g));assert(max(g)<=o.constraintTolerance);
 assert(max(abs(p.kappa))<=v.kappa_max*(1+1e-8));
 clearances(end+1)=m.smoothing.clearance; %#ok<AGROW>
 [ok,gaps]=parking.FootprintClearance([p.x p.y p.theta],c,m.smoothing.clearance);assert(ok);minGap=min(minGap,min(gaps));
 reference=last.reference;assert(max(abs(points-reference)-last.bubble_radii/sqrt(2),[],'all')<1e-6);
 assert(max(abs(points([1 end],:)-reference([1 end],:)),[],'all')<1e-7);
 speed=m.speed;h=diff(speed.t);
 ds=diff(speed.s)-h.*speed.v(1:end-1)-h.^2.*(speed.a(1:end-1)/3+speed.a(2:end)/6);
 dv=diff(speed.v)-h.*(speed.a(1:end-1)+speed.a(2:end))/2;
 maxDynamics=max(maxDynamics,max(abs([ds;dv])));assert(maxDynamics<1e-6);
 middle=speed.v(1:end-1)+h.*speed.a(1:end-1)/2;
 bounds=max([0;-speed.s;speed.s-p.s(end);-speed.v;speed.v-m.speed_solver.speed_cap;-middle;middle-m.speed_solver.speed_cap; ...
  abs(speed.a)-min(v.a_accel,v.a_brake);abs(diff(speed.a)./h)-o.maxJerk]);
 maxBounds=max(maxBounds,bounds);assert(bounds<1e-6);
 phi=atan(v.lw*p.kappa);
 maximumRate=independentPeak(p,speed,v.lw);assert(maximumRate<=v.wmax+1e-6);
 raw=m.speed_solver.native_speed;scale=m.speed_solver.clock_scale;
 assert(max(abs([speed.t-raw.t*scale;speed.v-raw.v/scale;speed.a-raw.a/scale^2;speed.jerk-raw.jerk/scale^3]))<1e-8);
 rows=find(q.t>=elapsed-1e-8&q.t<=elapsed+speed.t(end)+1e-8);t=q.t(rows)-elapsed;t=max(0,min(speed.t(end),t));
 index=discretize(t,[-Inf;speed.t(2:end-1);Inf]);tau=t-speed.t(index);jerk=speed.jerk(index);
 s=speed.s(index)+speed.v(index).*tau+.5*speed.a(index).*tau.^2+jerk.*tau.^3/6;s=max(0,min(p.s(end),s));
 vv=p.gear*(speed.v(index)+speed.a(index).*tau+.5*jerk.*tau.^2);aa=p.gear*(speed.a(index)+jerk.*tau);
 expected=[interp1(p.s,p.x,s),interp1(p.s,p.y,s),interp1(p.s,p.theta,s),vv,interp1(p.s,phi,s),aa];
 observed=[q.x(rows),q.y(rows),q.theta(rows),q.v(rows),q.phi(rows),q.a(rows)];
 error=observed-expected;error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));maxMapping=max(maxMapping,max(abs(error),[],'all'));assert(maxMapping<1e-6);
 if run>1,cusps(end+1)=rows(1);assert(abs(q.v(rows(1)))<1e-7);end %#ok<AGROW>
 elapsed=elapsed+speed.t(end);
 if run<numel(result.diagnostics.maneuvers)
  next=result.diagnostics.maneuvers{run+1}.path;elapsed=elapsed+abs(atan(v.lw*next.kappa(1))-phi(end))/v.wmax;
 end
end
elapsed=elapsed+abs(phi(end))/v.wmax;assert(abs(elapsed-q.t(end))<1e-7);
assert(all(diff(q.t)>0)&&max(abs([q.v([1 end]);q.phi([1 end])]))<1e-7);
assert(max(abs(q.phi))<=v.phimax+1e-7&&max(abs(q.omega))<=v.wmax+1e-7&&max(abs(diff(q.phi)./diff(q.t)))<=v.wmax+1e-6);
task=c.task;endpoint=[q.x(1)-task.x0;q.y(1)-task.y0;q.x(end)-task.xf;q.y(end)-task.yf;atan2(sin(q.theta([1 end])-[task.theta0;task.thetaf]),cos(q.theta([1 end])-[task.theta0;task.thetaf]))];assert(max(abs(endpoint))<1e-6);
ref=cpe.MakeReference(result,c);times=unique([(0:.001:ref.tf)';ref.tf]);dense=cpe.ReferenceAt(ref,times,v);[~,gap]=parking.FootprintClearance([dense.x,dense.y,dense.theta],c,0);
check=struct('passed',true,'maximum_normalized_quartic_residual',maxG,'minimum_path_node_separation',minGap,'maximum_speed_dynamics_residual',maxDynamics, ...
 'maximum_continuous_speed_bound_excess',maxBounds,'maximum_independent_export_error',maxMapping,'maximum_endpoint_error',max(abs(endpoint)), ...
 'exact_cusp_indices',cusps,'physical_margin_by_maneuver_m',clearances,'maximum_steering_rate',max(abs(q.omega)),'reference_collision_percent',100*mean(gap<=0),'reference_time_s',q.t(end));
end
function peak=independentPeak(p,speed,lw)
% Polynomial roots independently locate steering-slope boundary crossings.
slope=diff(atan(lw*p.kappa))./diff(p.s);peak=0;
for k=1:numel(speed.jerk)
 h=speed.t(k+1)-speed.t(k);j=speed.jerk(k);a=speed.a(k);v=speed.v(k);s0=speed.s(k);
 times=[0;h];targets=p.s(p.s>s0+1e-9&p.s<speed.s(k+1)-1e-9);
 for s=targets'
  rr=roots([j/6,a/2,v,s0-s]);inside=real(rr(abs(imag(rr))<1e-7&real(rr)>0&real(rr)<h));times=[times;inside]; %#ok<AGROW>
 end
 if abs(j)>1e-12&&-a/j>0&&-a/j<h,times(end+1)=-a/j;end
 times=sort(times);
 for ii=1:numel(times)-1
  tt=times(ii:ii+1);mid=mean(tt);s=s0+v*mid+a*mid^2/2+j*mid^3/6;
  s=max(p.s(1),min(p.s(end),s));
  row=max(1,min(numel(slope),find(p.s<=s,1,'last')));
  peak=max(peak,max(abs(v+a*tt+j*tt.^2/2))*abs(slope(row)));
 end
end
end
