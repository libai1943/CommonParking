function [z,ctx,raw]=Initialize(c,o)
raw=parking.SearchHybridAStar(c,o.search);z=[];ctx=[];if ~raw.success,return;end
r=cp.EmptyResult('initialization',c.id,'path');r.path=cp.PathFromArcs([c.task.x0 c.task.y0 c.task.theta0],raw.primitives,c.vehicle,.05);
ref=cpe.MakeReference(r,c);q=cpe.ReferenceAt(ref,linspace(0,ref.tf,o.nodes)',c.vehicle);
ctx=indpark.Context(c,o);ctx.goal(3)=q.theta(end);
ctx.reference=[q.x(1:end-1)+q.x(2:end),q.y(1:end-1)+q.y(2:end),q.theta(1:end-1)+q.theta(2:end)]'/2;
Y=[q.x,q.y,q.theta,q.v/3,.96*q.phi,zeros(o.nodes,5)]';z=[Y(:);log(3*ref.tf)];z=indpark.GuessCostates(z,ctx);
end
