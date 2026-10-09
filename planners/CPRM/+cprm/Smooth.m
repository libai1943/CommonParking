function [pieces,info]=Smooth(route,c,o)
pieces=struct([]);info=struct('attempted_runs',0,'accepted_runs',0,'knot_iterations',0,'raw_length',sum(arrayfun(@(x)sum(abs(x.primitives(:,1))),route)));
j=1;
while j<=numel(route)
 last=j;direction=sign(route(j).primitives(1,1));
 if route(j).via>0
  while last<numel(route)&&route(last+1).via>0&&sign(route(last+1).primitives(1,1))==direction,last=last+1;end
 end
 accepted=false;
 if route(j).via>0&&last-j>=1
  control=[route(j).start(1:2);vertcat(route(j:last).control);route(last).goal(1:2)];
  spans=size(control,1)-3;knots=[zeros(1,3),linspace(0,1,spans+1),ones(1,3)];best=struct([]);bestPeak=inf;info.attempted_runs=info.attempted_runs+1;
  for iteration=0:o.splineIterations
   candidate=cprm.SplinePieces(control,knots,direction);peak=max([candidate.maximum_curvature]);
   if any([candidate.minimum_speed]<=o.minimumSpeed)||~isfinite(peak),break;end
   if peak>=bestPeak-o.splineImprovement,break;end
   best=candidate;bestPeak=peak;
   if iteration==o.splineIterations,break;end
   width=diff(unique(knots));proposed=max(1e-6,[candidate.maximum_curvature]).*width;proposed=proposed/sum(proposed);
   knots=[zeros(1,3),0,cumsum(proposed),ones(1,3)];knots(end-3:end)=1;info.knot_iterations=info.knot_iterations+1;
  end
  if ~isempty(best)&&bestPeak<=c.vehicle.kappa_max
   clear=true;for span=1:numel(best),if ~cprm.CubicFree(best(span),c,o),clear=false;break;end,end
   if clear,pieces=[pieces,best];accepted=true;info.accepted_runs=info.accepted_runs+1;end %#ok<AGROW>
  end
 end
 if ~accepted
  for edge=j:last
   q=route(edge).start;
   for k=1:size(route(edge).primitives,1)
    pp=route(edge).primitives(k,:);p=cprm.Piece('arc',sign(pp(1)));p.start=q;p.primitive=pp;p.length=abs(pp(1));p.maximum_curvature=abs(pp(2));p.minimum_speed=p.length;
    pieces=[pieces,p];q=parking.IntegratePrimitive(q,pp(1),pp(2)); %#ok<AGROW>
   end
  end
 end
 j=last+1;
end
for j=1:numel(pieces),pieces(j)=cprm.Prepare(pieces(j));end
info.final_length=sum([pieces.length]);
end
