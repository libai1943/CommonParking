function report=TestTEB()
SetupCommonParking();state=rng;restore=onCleanup(@()rng(state));rng(178); %#ok<NASGU>
o=teb.Config();c=LoadCase(9);o.vehicle=c.vehicle;env=teb.Prepare(c);
% Exact exterior distance against axis-aligned reference and SAT sign for rotation.
q=[4*randn(50,2),zeros(50,1)];d=teb.Distance(q,env);v=c.vehicle;
polygons=struct('count',numel(env.polygons),'vertices',{env.polygons});
boxes=[q(:,1)-v.lr,q(:,1)+v.lw+v.lf,q(:,2)-v.lb/2,q(:,2)+v.lb/2];reference=cp.BoxObstacleDistance(boxes,polygons);
distanceError=max(abs(max(0,min(d,[],2))-reference));assert(distanceError<1e-9);
q(:,3)=randn(50,1);d=teb.Distance(q,env);[~,sat]=parking.FootprintClearance(q,c,0);
assert(isequal(min(d,[],2)>0,sat>0));
% Graph line intersections, including tangencies and a point inside a polygon.
simple=env;simple.polygons={[-1 -1;1 -1;1 1;-1 1]};simple.markers=0;simple.residues=1;
assert(~teb.SegmentFree([-2 0],[2 0],simple)&&teb.SegmentFree([-2 2],[2 2],simple));
assert(~teb.SegmentFree([0 0],[0 0],simple)&&teb.SegmentFree([2 0],[2 0],simple));
upper=[-2 0;-2 2;2 2;2 0];lower=[-2 0;-2 -2;2 -2;2 0];
h1=teb.Signature(upper,simple);h2=teb.Signature(lower,simple);
assert(abs(abs(h1-h2)-2*pi)<1e-10&&abs(teb.Signature(flipud(upper),simple)+h1)<1e-10);
refined=[upper(1,:);mean(upper(1:2,:));upper(2:end,:)];assert(abs(teb.Signature(refined,simple)-h1)<1e-10);
graphCase=c;graphCase.task=struct('x0',-3,'y0',0,'theta0',0,'xf',3,'yf',0,'thetaf',0);
graphOptions=o;graphOptions.samples=50;graphOptions.maximumClasses=2;
empty=struct('pose',{},'dt',{},'cost',{},'signature',{},'origin',{},'solver',{});
[routes,graphReport]=teb.Explore(empty,graphCase,simple,graphOptions);
assert(numel(routes)==2&&abs(routes(1).signature-routes(2).signature)>1);
% Colored sparse derivative must equal an independently evaluated dense derivative.
p=[linspace(1,5,9)',randn(9,1),.1*randn(9,1)];b=teb.Initialize(p,o,'test');b.dt=.2+.1*rand(8,1);
z=teb.Pack(b);r=teb.Residual(z,b,env,o);[pattern,colors]=teb.Pattern(9,numel(env.polygons));
J=teb.Jacobian(z,r,b,env,o,pattern,colors);dense=zeros(numel(r),numel(z));
for k=1:numel(z),h=o.finiteDifferenceStep*(1+abs(z(k)));zz=z;zz(k)=zz(k)+h;dense(:,k)=(teb.Residual(zz,b,env,o)-r)/h;end
jacobianError=max(abs(full(J)-dense),[],'all');assert(jacobianError<1e-6);
for color=1:11,assert(all(sum(pattern(:,colors==color),2)<=1));end
% Common model: exact straight trapezoidal states and rest boundaries.
clearEnv=env;clearEnv.polygons={};p=[0 0 0;.5 0 0;1.5 0 0;2 0 0];b=teb.Initialize(p,o,'test');
b.pose(:,4)=[0;1;1;0];b.pose(:,5)=0;b.dt=ones(3,1);
[r,parts]=teb.Residual(teb.Pack(b),b,clearEnv,o);expected=3;
assert(abs(r'*r-expected)<1e-12&&max(abs(parts.dynamics),[],'all')==0);
[optimized,info]=teb.Optimize(b,clearEnv,o);assert(info.success&&optimized.cost<expected);
% A common heading lift and a rotation of the scene leave the objective unchanged.
lifted=b;lifted.pose(:,3)=lifted.pose(:,3)+4*pi;
rl=teb.Residual(teb.Pack(lifted),lifted,clearEnv,o);assert(abs(rl'*rl-expected)<1e-10);
turn=.71;rot=[cos(turn) -sin(turn);sin(turn) cos(turn)];rotated=b;
rotated.pose(:,1:2)=b.pose(:,1:2)*rot';rotated.pose(:,3)=b.pose(:,3)+turn;
rr=teb.Residual(teb.Pack(rotated),rotated,clearEnv,o);assert(abs(rr'*rr-expected)<1e-10);
backward=b;backward.pose(:,1)=-b.pose(:,1);backward.pose(:,4)=-b.pose(:,4);
rb=teb.Residual(teb.Pack(backward),backward,clearEnv,o);assert(abs(rb'*rb-expected)<1e-10);
% Turning is v*tan(phi)/wheelbase, not v*sin(phi) or a yaw-rate surrogate.
curve=b;curve.dt(:)=.4;curve.pose(:,4)=.8;curve.pose(:,5)=.35;
curve.pose(:,3)=(0:3)'*.4*.8*tan(.35)/v.lw;
for k=1:3
 curve.pose(k+1,1:2)=curve.pose(k,1:2)+.4*.8/2*([cos(curve.pose(k,3)),sin(curve.pose(k,3))]+[cos(curve.pose(k+1,3)),sin(curve.pose(k+1,3))]);
end
[~,parts]=teb.Residual(teb.Pack(curve),curve,clearEnv,o);assert(max(abs(parts.dynamics),[],'all')<1e-14);
[q,export]=teb.Export(curve,clearEnv,o);assert(export.success&&isequal(q.phi,curve.pose(:,5))&&max(abs(q.omega))==0);
% Boundary/cusp states are explicit, not ratios of averaged yaw and speed.
b=teb.Initialize([linspace(0,3,11)',zeros(11,2)],o,'test');
% Resizing must preserve endpoints and sum(dt), even when splitting/merging.
for trial=1:20
 b.dt=.05+.7*rand(10,1);oldTime=sum(b.dt);oldEndpoints=b.pose([1 end],:);next=teb.Resize(b,o);
 assert(abs(sum(next.dt)-oldTime)<1e-10&&max(abs(next.pose([1 end],:)-oldEndpoints),[],'all')<1e-10);
end
report=struct('passed',true,'common_model_checks',true,'exact_distance_error',distanceError,'colored_jacobian_error',jacobianError, ...
 'homology_class_separation',abs(h1-h2),'exploration',graphReport,'straight_initial_cost',expected,'straight_lm_cost',optimized.cost);disp(report);
end
