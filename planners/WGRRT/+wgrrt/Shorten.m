function [primitives,info]=Shorten(start,primitives,c,options,connection)
clock=tic;initialLength=sum(abs(primitives(:,1)));length=initialLength;failures=0;accepted=0;attempts=0;
cache=arcCache(start,primitives);
for iteration=1:options.shortcutIterations
 if failures>=options.shortcutFailures||toc(clock)>options.shortcutSeconds,break;end
 attempts=attempts+1;events=sort(rand(2,1))*length;q=cachedPose(cache,primitives,events);
 candidate=wgrrt.Shortest(connection,q(1,:),q(2,:),c.vehicle.kappa_max);newLength=sum(abs(candidate(:,1)));
 if diff(events)-newLength>options.improvementTolerance&&wgrrt.ArcsFree(q(1,:),candidate,c)
  before=parking.SlicePrimitives(primitives,0,events(1));after=parking.SlicePrimitives(primitives,events(2),length);
  primitives=[before;candidate;after];length=sum(abs(primitives(:,1)));accepted=accepted+1;failures=0;cache=arcCache(start,primitives);
 else,failures=failures+1;
 end
end
info=struct('initial_length',initialLength,'final_length',length,'attempts',attempts,'accepted_shortcuts',accepted,'consecutive_failures',failures,'time_s',toc(clock));
end
function cache=arcCache(start,pp)
turn=pp(:,1).*pp(:,2);theta=start(3)+[0;cumsum(turn(1:end-1))];
dx=pp(:,1).*cos(theta);dy=pp(:,1).*sin(theta);curved=abs(pp(:,2))>1e-12;
dx(curved)=(sin(theta(curved)+turn(curved))-sin(theta(curved)))./pp(curved,2);
dy(curved)=-(cos(theta(curved)+turn(curved))-cos(theta(curved)))./pp(curved,2);
cache.q=[start(1)+[0;cumsum(dx(1:end-1))],start(2)+[0;cumsum(dy(1:end-1))],theta];
cache.ends=cumsum(abs(pp(:,1)));cache.starts=[0;cache.ends(1:end-1)];
end
function q=cachedPose(cache,pp,s)
index=discretize(s,[-inf;cache.ends(1:end-1);inf]);q=zeros(numel(s),3);
for j=1:numel(s)
 i=index(j);ds=sign(pp(i,1))*(s(j)-cache.starts(i));q(j,:)=parking.IntegratePrimitive(cache.q(i,:),ds,pp(i,2));q(j,3)=cache.q(i,3)+ds*pp(i,2);
end
end

