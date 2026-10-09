function [branches,info]=Search(start,goal,v,body,polygons,o)
timer=tic;points=[start(1:2);goal(1:2);vertcat(polygons{:})];bbox=[min(points(:,1))-o.padding,max(points(:,1))+o.padding,min(points(:,2))-o.padding,max(points(:,2))+o.padding];
T={struct('q',start,'parent',0,'u',[0,0],'duration',0),struct('q',goal,'parent',0,'u',[0,0],'duration',0)};signs=[1,-1];attempts={};branches=[];tested=zeros(0,2);info=struct('success',false,'nodes',[],'seconds',0,'attempts',{{}},'best_gap',inf,'iterations',0);
while toc(timer)<o.seconds && size(T{1}.q,1)+size(T{2}.q,1)<o.maxNodes && numel(attempts)<o.maxAttempts
 for tree=1:2
  [T{tree},added]=kdf.Expand(T{tree},T{3-tree},v,body,polygons,bbox,signs(tree),o);info.iterations=info.iterations+1;
  if ~added,continue;end
  last=size(T{tree}.q,1);[d,id]=min(kdf.Distance(T{3-tree}.q,T{tree}.q(last,:),o.scale));info.best_gap=min(d,info.best_gap);
  if d>o.joinTolerance,continue;end
  pair=[last,id];if tree==2,pair=fliplr(pair);end
  if ismember(pair,tested,'rows')||any(pair==1),continue;end
  tested(end+1,:)=pair;[U1,h1]=kdf.Extract(T{1},pair(1));[U2,h2]=kdf.Extract(T{2},pair(2));
  [V1,V2,di]=kdf.Deform(start,goal,U1,h1,U2,h2,v,body,polygons,o);di.pair=pair;di.initial_input={U1,U2};di.durations={h1,h2};attempts{end+1}=di;
  fprintf('KDF nodes %d+%d gap %.4g deform %d residual %.3g\n',size(T{1}.q,1),size(T{2}.q,1),d,di.success,max(abs(di.final_gap)));
  if di.success,branches=struct('roots',[start;goal],'inputs',{{V1,V2}},'durations',{{h1,h2}});info.success=true;break;end
 end
 if info.success,break;end
end
info.nodes=[size(T{1}.q,1),size(T{2}.q,1)];info.seconds=toc(timer);info.attempts=attempts;info.trees=T;
end
