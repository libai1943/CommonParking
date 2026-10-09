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
report=struct('passed',true,'signed_area_affine_error',maximumError,'random_rotational_envelopes',40,'shoelace_cancellation_retained',true);disp(report);
end
