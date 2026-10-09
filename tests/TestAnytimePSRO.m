function report=TestAnytimePSRO()
c=LoadCase(1);v=c.vehicle;o=psro.Config();polygon=[-2 -1;3 -1;3 2;-2 2];point=[4 1];
[plane,signs]=psro.AreaPlane(point,polygon,o.areaExcess);maximumError=0;
original=rng;restore=onCleanup(@()rng(original));rng(153); %#ok<NASGU>
for j=1:100
 P=randn(1,2)*4;next=polygon([2:end 1],:);tri=.5*((polygon(:,1)-P(1)).*(next(:,2)-P(2))-(polygon(:,2)-P(2)).*(next(:,1)-P(1)));
 signedExcess=sum(signs.*tri)-polyarea(polygon(:,1),polygon(:,2))-o.areaExcess;
 linear=plane(1:2)*P'-plane(3);maximumError=max(maximumError,abs(signedExcess-linear));assert(abs(signedExcess-linear)<1e-12);
 assert(sum(abs(tri))+1e-12>=sum(signs.*tri));
end
body=[v.lw+v.lf v.lb/2;v.lw+v.lf -v.lb/2;-v.lr -v.lb/2;-v.lr v.lb/2];
for j=1:40
 pose=[randn(1,2),10*randn];r=rand;delta=rand*pi;box=psro.FootprintBox(pose,r,delta,body);
 for a=linspace(pose(3)-delta,pose(3)+delta,200)
  R=[cos(a) -sin(a);sin(a) cos(a)];corners=body*R'+pose(1:2);assert(all(corners-r>=box(1:2)-1e-10,'all')&&all(corners+r<=box(3:4)+1e-10,'all'));
 end
end
path=struct('x',[0;1;2],'y',[0;0;0]);q=struct('x',[0;1;2],'y',[0;1;0]);assert(abs(psro.PathGap(path,q)-1)<1e-12);
% A closed-loop area can cancel; the literal shoelace gap is retained.
path=struct('x',[0;1;2;3;4],'y',zeros(5,1));q=struct('x',path.x,'y',[0;1;0;-1;0]);assert(psro.PathGap(path,q)==0);
% Uniform time intervals are part of the NLP, including for stationary motion.
stationary=c;stationary.task=struct('x0',0,'y0',0,'theta0',0,'xf',0,'yf',0,'thetaf',0);
q=struct('t',[0;.1;.2],'x',zeros(3,1),'y',zeros(3,1),'theta',zeros(3,1),'v',zeros(3,1),'phi',zeros(3,1),'a',zeros(3,1),'omega',zeros(3,1));
data=struct('position_radius',ones(3,1),'angle_radius',ones(3,1),'ego_planes',zeros(0,6),'obstacle_planes',zeros(0,6));
assert(psro.Check(stationary,q,[.1;.1],data,q,o).success);
assert(~psro.Check(stationary,q,[.1;.11],data,q,o).success);
report=struct('passed',true,'signed_area_affine_error',maximumError,'random_rotational_envelopes',40,'shoelace_cancellation_retained',true,'uniform_mesh_checked',true);disp(report);
end
