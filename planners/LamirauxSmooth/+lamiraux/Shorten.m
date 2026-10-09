function [pieces,info]=Shorten(pieces,c,o)
clock=tic;initial=sum([pieces.length]);length=initial;attempts=0;failures=0;accepted=0;
for iteration=1:o.shortcutIterations
 if failures>=o.shortcutFailures||toc(clock)>o.shortcutSeconds,break;end
 attempts=attempts+1;events=sort(rand(2,1))*length;q=lamiraux.PathPose(pieces,events);
 [candidate,ok]=lamiraux.Steer(q(1,:),q(2,:),c,o);
 if ok&&sum([candidate.length])<diff(events)-o.improvementTolerance
  pieces=[lamiraux.Slice(pieces,0,events(1)),candidate,lamiraux.Slice(pieces,events(2),length)];
  length=sum([pieces.length]);accepted=accepted+1;failures=0;
 else,failures=failures+1;
 end
end
info=struct('initial_length',initial,'final_length',length,'attempts',attempts,'accepted_shortcuts',accepted,'consecutive_failures',failures,'time_s',toc(clock));
end
