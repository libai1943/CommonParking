function cost=PathCost(q,pp,route,o,kappa)
% Independent cost recomputation over an exported primitive sequence.
cost=0;previous=0;
for j=1:size(pp,1)
 [circle,~]=osehs.Map(q,route,kappa);direction=sign(pp(j,1));preferred=route.direction(circle);
 multiplier=1;if preferred~=0&&direction~=preferred,multiplier=o.wrongDirectionFactor;end
 penalty=o.cuspPenalty;if preferred==0,penalty=o.bidirectionalCuspPenalty;end
 cost=cost+multiplier*abs(pp(j,1))+penalty*(previous~=0&&direction~=previous);
 q=parking.IntegratePrimitive(q,pp(j,1),pp(j,2));previous=direction;
end
end
