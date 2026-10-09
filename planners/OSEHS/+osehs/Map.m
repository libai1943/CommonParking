function [index,h]=Map(q,route,kappa)
distance=osehs.Metric(route.q,q,kappa);least=min(distance);index=find(distance<=least+1e-12,1,'last');
next=min(index+1,size(route.q,1));h=osehs.Metric(q,route.q(next,:),kappa)+route.remaining(next);
end
