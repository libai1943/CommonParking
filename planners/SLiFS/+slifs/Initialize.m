function [z,info]=Initialize(c,options)
% Remark 3's Hybrid A* alternative; no CG smoothing or optimizer warm start.
raw=parking.SearchHybridAStar(c,options.search);info=struct('success',raw.success,'search',raw);z=[];
if ~raw.success,return;end
seed=cp.EmptyResult('SLiFS_initial_path',c.id,'path');t=c.task;
seed.path=cp.PathFromArcs([t.x0 t.y0 t.theta0],raw.primitives,c.vehicle,.05);
reference=cpe.MakeReference(seed,c);duration=max(options.initialTime,reference.tf);
sample=cpe.ReferenceAt(reference,linspace(0,reference.tf,options.nodes)',c.vehicle);h=duration/(options.nodes-1);
sample.v=reshapeSignal(sample.v*reference.tf/duration,c.vehicle.vmax,c.vehicle.amax*h);
sample.phi=reshapeSignal(sample.phi,c.vehicle.phimax,c.vehicle.wmax*h);
offset=c.vehicle.length-c.vehicle.length/(2*options.circles)-c.vehicle.lr;
q=[sample.x sample.y sample.theta sample.v sample.phi,sample.x+offset*cos(sample.theta),sample.y+offset*sin(sample.theta)];
z=[q(:);duration];info.initial_time_s=duration;
end
function y=reshapeSignal(y,limit,change)
y=max(-limit,min(limit,y));y([1 end])=0;
for i=2:numel(y),y(i)=max(y(i-1)-change,min(y(i-1)+change,y(i)));end
y(end)=0;
for i=numel(y)-1:-1:1,y(i)=max(y(i+1)-change,min(y(i+1)+change,y(i)));end
end
