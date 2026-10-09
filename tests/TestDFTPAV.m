function report=TestDFTPAV()
% Check the analytical adjoint against complete finite differences, including
% active physical/body penalties and all movable gear boundary variables.
cfg=BenchmarkConfig();c=LoadCase(3);c.obstacle=struct('num_obs',0,'obs',{{}});c.task.x0=.2;c.task.y0=-.3;c.task.theta0=.4;
path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],[2 .08;-1.5 -.05],cfg.vehicle,.05);c.task.xf=path.x(end);c.task.yf=path.y(end);c.task.thetaf=path.theta(end);
o=dftpav.Config();o.samplesPerPiece=5;[z,p]=dftpav.Prepare(path,c,o);z=z+.015*sin((1:numel(z))');
p.caseData.vehicle.vmax=.2;p.caseData.vehicle.amax=.05;p.caseData.vehicle.kappa_max=.001;
for i=1:numel(p.segments),for j=1:numel(p.segments{i}.corridors),p.segments{i}.corridors{j}.b=p.segments{i}.corridors{j}.b-2.15;end,end
[f,g,detail]=dftpav.Objective(z,p);finite=zeros(size(z));
for k=1:numel(z)
 step=2e-6*max(1,abs(z(k)));plus=z;minus=z;plus(k)=plus(k)+step;minus(k)=minus(k)-step;
 finite(k)=(dftpav.Objective(plus,p)-dftpav.Objective(minus,p))/(2*step);
end
error=max(abs(g-finite)./max(1,abs(g)+abs(finite)));assert(error<2e-5,'DFTPAV:Gradient','Analytic adjoint relative error %g.',error);
joinError=0;for i=1:numel(detail.segments)
 seg=detail.segments{i};for j=1:seg.M-1,for d=0:4
  a=dftpav.Basis(1,d)*seg.coefficients((j-1)*6+(1:6),:);b=dftpav.Basis(0,d)*seg.coefficients(j*6+(1:6),:);joinError=max(joinError,max(abs(a-b)));
 end,end
end
assert(joinError<1e-8);[~,full]=dftpav.MinimumJerk(zeros(0,2),[0 0],[1 0],[0 0],[0 0],1);C=full.A\full.B;assert(norm(C(:,1)-[0;0;0;10;-15;6])<1e-10);
tau=[-2;-.1;0;.1;2];[T,dT]=dftpav.TimeMap(tau);fd=(dftpav.TimeMap(tau+1e-6)-dftpav.TimeMap(tau-1e-6))/2e-6;assert(max(abs(dT-fd))<1e-8&&all(T>0));
[sample,native]=dftpav.Export(detail,cfg.vehicle);assert(numel(sample.cusp_indices)==3&&numel(unique(sample.gear))==2);
assert(max(abs(abs(native.v([1 end]))-.05))<1e-7);assert(max(abs([sample.x(1)-c.task.x0;sample.y(1)-c.task.y0;sample.x(end)-c.task.xf;sample.y(end)-c.task.yf]))<1e-8);
report=struct('passed',true,'objective',f,'full_adjoint_relative_error',error,'C4_join_error',joinError,'minimum_jerk_quintic_verified',true,'time_map_verified',true,'exact_cusp_export_verified',true);disp(report);
end
