function [path,info,detail]=Smooth(raw,c,centres,bounds,o)
% The article leaves cusp discretization unspecified: retain their poses and
% fix the two adjacent offset nodes on each side of every fixed endpoint.
reference=cell(numel(raw.cusp_indices)-1,1);count=0;
for j=1:numel(reference)
 left=raw.s(raw.cusp_indices(j));right=raw.s(raw.cusp_indices(j+1));
 n=max(6,round(o.smoothingNodes*(right-left)/raw.s(end)));
 q=cp.ArcPose(raw.geometry.start,raw.geometry.primitives,linspace(left,right,n)');nodes=(3:n-2)';
 reference{j}=struct('q',q,'gear',raw.gear(raw.cusp_indices(j)),'variable_nodes',nodes,'variables',count+(1:numel(nodes))');count=count+numel(nodes);
end
z0=zeros(count,1);previous=maxNumCompThreads(1);restore=onCleanup(@()maxNumCompThreads(previous)); %#ok<NASGU>
timer=tic;lastZ=[];lastPieces=[];lastObjective=[];lastInequality=[];
[~,initialObjective,initialConstraint]=dgrid.Geometry(z0,reference,c,centres,bounds);
settings=optimoptions('fmincon','Algorithm','sqp','Display','off','MaxIterations',o.smoothingIterations,'MaxFunctionEvaluations',100000, ...
 'ConstraintTolerance',o.constraintTolerance,'OptimalityTolerance',1e-5,'StepTolerance',1e-8,'OutputFcn',@stop);
[z,value,flag,output]=fmincon(@objective,z0,[],[],[],[],[],[],@constraints,settings);
[pieces,value,con]=dgrid.Geometry(z,reference,c,centres,bounds);
accepted=flag>0&&max(con)<=o.constraintTolerance&&value<=initialObjective+1e-8;
info=struct('accepted',accepted,'exit_flag',flag,'iterations',output.iterations,'message',output.message, ...
 'seconds',toc(timer),'initial_objective',initialObjective,'objective',value,'max_constraint',max(con), ...
 'initial_max_constraint',max(initialConstraint),'variables',count);
detail=struct('reference',{reference},'offset_variables',z,'pieces',{pieces});
if accepted,path=dgrid.Export(pieces,c.vehicle,o.outputSpacing);else,path=raw;end
 function update(z)
  if ~isequal(z,lastZ),[lastPieces,lastObjective,lastInequality]=dgrid.Geometry(z,reference,c,centres,bounds);lastZ=z;end
 end
 function f=objective(z),update(z);f=lastObjective;end
 function [g,h]=constraints(z),update(z);g=lastInequality;h=[];end
 function done=stop(~,~,~),done=toc(timer)>o.smoothingSeconds;end
end
