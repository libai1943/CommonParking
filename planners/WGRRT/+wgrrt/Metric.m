function value=Metric(q,target,o)
delta=q-target;delta(:,3)=atan2(sin(delta(:,3)),cos(delta(:,3)));delta(:,3)=o.headingMetricScale*delta(:,3);value=vecnorm(delta,2,2);
end
