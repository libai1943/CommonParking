function report=TestSteeringJoins()
% Initial/final alignment and reverse joins must spend physical time at rest.
SetupCommonParking();cfg=BenchmarkConfig();w=cfg.vehicle.wmax;reports=cell(2,1);
for method=1:2
 if method==1,join=@ritp.Join;else,join=@dliaps.Join;end
 one=struct('t',[0;1;2],'x',[0;.5;1],'y',[0;0;0],'theta',[.7;.7;.7], ...
  'v',[0;1;0],'phi',[.2;.25;.3],'a',[0;0;0],'omega',[.05;.05;0]);
 two=one;two.x=[1;.5;0];two.v=-one.v;two.theta=two.theta+2*pi;two.phi=[-.2;-.15;-.1];
 q=join([],one,w);q=join(q,two,w);q=join(q,[],w);
 expected=4+(abs(.2)+abs(-.2-.3)+abs(-.1))/w;
 assert(abs(q.t(end)-expected)<1e-12&&all(diff(q.t)>0));
 assert(max(abs(q.theta-.7))<1e-12&&max(abs(q.phi([1 end])))<1e-12);
 assert(max(abs(diff(q.phi)./diff(q.t)))<=w+1e-12);
 stop=abs(diff(q.x))<1e-12;assert(nnz(stop)==3);
 assert(max(abs(q.v(find(stop))))==0&&max(abs(q.v(find(stop)+1)))==0);
 reports{method}=struct('duration',q.t(end),'steering_only_intervals',nnz(stop),'maximum_rate',max(abs(diff(q.phi)./diff(q.t))));
end
report=struct('passed',true,'methods',{reports});disp(report);
end
