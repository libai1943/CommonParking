function [h,source]=Heuristic(q,target,B,c,o)
near=find(biagt.Distance(q,B.q(1:B.n,:),o.metric)<=o.gamma);source=0;
if isempty(near),h=o.rho*biagt_rs_mex(q,target,c.vehicle.kappa_max);return;end
cost=o.rho*biagt_rs_mex(q,B.q(near,:),c.vehicle.kappa_max)+B.g(near);[h,k]=min(cost);source=near(k);
end
