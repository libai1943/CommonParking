function check=MeshCheck(z,points,v,o)
N=(numel(z)-1)/5;h=z(end)/(N-1);sub=max(8,ceil(h/o.checkStep));t=linspace(0,z(end),(N-1)*sub+1);
[q,d]=ppocp.Dense(z,v,t);f=ppocp.Model(q,v.lw);defect=max(abs(d(1:3,:)-f),[],'all');potential=max(ppocp.Potential(q(1:3,:),points,v));
bound=max([0,max(abs(q(4,:)))-v.vmax,max(abs(q(5,:)))-v.phimax,max(abs(d(4,:)))-v.amax,max(abs(d(5,:)))-v.wmax]);
check=struct('passed',defect<=o.meshTolerance&&potential<=o.constraintTolerance&&bound<=o.constraintTolerance, ...
    'max_ode_defect',defect,'max_point_potential',potential,'max_bound_violation',bound, ...
    'samples',numel(t),'step_s',h/sub);
end
