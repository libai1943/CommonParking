function report=TestKinoDeform()
SetupCommonParking();ccp.EnsureNative();c=LoadCase(3);v=c.vehicle;o=kdf.Config(v);old=rng;rng(110);restore=onCleanup(@()rng(old)); %#ok<NASGU>
body=[v.lw+v.lf,v.lr,v.lb/2];jacobian=0;odeError=0;
for n=1:40
 q=[randn(1,2),3*randn,(2*rand-1)*1.5,(2*rand-1)*.4];u=[(2*rand-1)*.5,(2*rand-1)*.2];sgn=(-1)^n;h=.1;
 [out,A,B]=kdf.Step(q,u,h,sgn,v.lw);J=[A,B];fd=zeros(5,7);
 for k=1:7
  z=[q,u];zp=z;zm=z;zp(k)=zp(k)+1e-6;zm(k)=zm(k)-1e-6;
  fd(:,k)=(kdf.Step(zp(1:5),zp(6:7),h,sgn,v.lw)-kdf.Step(zm(1:5),zm(6:7),h,sgn,v.lw))'/2e-6;
 end
 jacobian=max(jacobian,max(abs(fd-J),[],'all'));
 [~,truth]=ode113(@(~,z)sgn*[z(4)*cos(z(3));z(4)*sin(z(3));z(4)*tan(z(5))/v.lw;u'],[0,h],q',odeset(RelTol=1e-12,AbsTol=1e-13));
 odeError=max(odeError,max(abs(out-truth(end,:))));
end
assert(jacobian<1e-8&&odeError<1e-7);
q=[0,0,.3,.4,.1];U=[.2,.05;-.1,-.1;.05,.06];h=[.7;.9;1.1];target=[1.1,.4,.35,.5,.08];polygons={[-2,-3;5,-3;5,-1.26;-2,-1.26]};
[~,g,J,Q]=kdf.Potential(q,U,h,1,target,v,body,polygons,o);gap=cc_steer_mex('clearance',Q(:,1:3),body,polygons,1);assert(min(gap)>0&&min(gap)<o.potentialRange);
fd=zeros(6,1);fdJ=zeros(5,6);
for i=1:6
 up=U;um=U;j=ceil(i/2);k=mod(i-1,2)+1;up(j,k)=up(j,k)+1e-6;um(j,k)=um(j,k)-1e-6;
 fd(i)=(kdf.Potential(q,up,h,1,target,v,body,polygons,o)-kdf.Potential(q,um,h,1,target,v,body,polygons,o))/2e-6;
 A=kdf.Rollout(q,up,h,1,v,o.deformStep);B=kdf.Rollout(q,um,h,1,v,o.deformStep);fdJ(:,i)=(A(end,:)-B(end,:))'/2e-6;
end
gradient=max(abs(fd-g));sensitivity=max(abs(fdJ-J),[],'all');assert(gradient<1e-5&&sensitivity<1e-7);
q0=[0,0,0,0,0];U=[.2,.1;.2,-.1;-.2,-.1;-.2,.1];h=ones(4,1);Q=kdf.Rollout(q0,U,h,1,v,.01);qg=Q(end,:);polygons={[-10,-10;-9,-10;-9,-9;-10,-9]};fine=o;fine.deformStep=.01;fine.finalTolerance=1e-10;
[U1,U2,info]=kdf.Deform(q0,qg,U(1:2,:)+[.008,-.009;-.01,.007],h(1:2),flipud(U(3:4,:)),h(1:2),v,body,polygons,fine);
assert(info.success);assert(all(info.cost(:,3)<info.cost(:,2)));b=struct('roots',[q0;qg],'inputs',{{U1,U2}},'durations',{{h(1:2),h(1:2)}});[z,join]=kdf.Output(b,v,o);
assert(max(abs(join).*o.scale)<1e-8);assert(all(diff(z.t)>0)&&abs(z.t(end)-4)<1e-12&&abs(z.v(1))+abs(z.v(end))<1e-12);
assert(max(abs([z.x(1),z.y(1),z.theta(1),z.v(1),z.phi(1)]-q0))<1e-12);assert(max(abs([z.x(end),z.y(end),z.theta(end),z.v(end),z.phi(end)]-qg))<1e-12);
q=[0,0,0,2,0];finish=kdf.Step(q,[0,0],3,1,v.lw);thin={[4.9,-.1;4.91,-.1;4.91,.1;4.9,.1]};assert(~kdf.Free([q;finish],[0;3],[0,0],1,body,thin,v,o));
report=struct('passed',true,'random_steps',40,'RK4_Jacobian_error',jacobian,'RK4_ODE_error',odeError,'potential_gradient_error',gradient,'terminal_sensitivity_error',sensitivity,'deformation_iterations',info.iterations,'deformation_join_error',max(abs(join)),'thin_obstacle_rejected',true);disp(report);
end
