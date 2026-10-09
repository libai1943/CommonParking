function phase=PathPhase(reference,gear,c,o)
e=ones(size(reference,1),1);phase=struct('success',false,'code','collision_iteration_limit', ...
 'reference',reference,'gear',gear,'iterations',0,'history',{{}});
for iteration=1:o.maximumIterations
 [coeff,solver,problem]=ritp.PathQP(reference,gear,e,o);phase.iterations=iteration;
 phase.solver=solver;phase.problem=problem;phase.coefficients=coeff;
 if ~solver.success,phase.code='path_qp_failed';return;end
 g=ritp.Geometry(coeff,problem.s,problem.L,gear,c.vehicle.lw);
 if any(~isfinite([g.theta;g.phi]))||min(g.parameter_speed)<o.derivativeTolerance
  phase.code='singular_polynomial';return;
 end
 [index,obstacle]=ritp.VertexCollision([g.x g.y g.theta],c);
 phase.history{iteration}=struct('collision_sample',index,'obstacle',obstacle,'qp',solver);
 phase.geometry=g;
 if index==0,phase.success=true;phase.code='solved';return;end
 [~,nearest]=min(sum((reference(:,1:2)-[g.x(index) g.y(index)]).^2,2));
 % z is unspecified in the paper; use the author's iteration-sized window.
 window=max(1,nearest-iteration):min(size(reference,1),nearest+iteration);
 e(window)=o.beta*e(window);phase.history{iteration}.reference_index=nearest;
end
end
