function report=TestVPF()
% Independent equation checks, including a limitation of the printed cover.
c=LoadCase(1);v=c.vehicle;o=vpf.Config();
assert(vpf.SegmentScore([0 0],[1 1],[0 1],[1 0])<0);
assert(vpf.SegmentScore([0 0],[1 0],[0 1],[1 1])>0);
assert(vpf.SegmentScore([0 0],[1 0],[2 0],[3 0])==0);
z=[1 2 .7 .8];u=[.3 -.2];h=.03;step=vpf.Step(z,u,h,v.lw);
k=tan(u(2))/v.lw;ds=z(4)*h+.5*u(1)*h^2;
exact=z;exact(1:2)=z(1:2)+[integral(@(s)cos(z(3)+k*s),0,ds),integral(@(s)sin(z(3)+k*s),0,ds)];
exact(3)=z(3)+k*ds;exact(4)=z(4)+h*u(1);rkError=max(abs(step-exact));assert(rkError<1e-9);
vv=[.7;-.9;.5;-.8];aa=[.1;.2;-.1;-.2];kk=[.2;-.2;-.1;.1];alpha=1.07;h=.3;
x=[v.lw+v.lf,v.lw+v.lf,v.lr,v.lr];y=v.lb/2*[1 -1 -1 1];signs=[1 -1 -1 1];
radius=sqrt((1-kk.*y).^2+(kk.*x).^2)./kk;
enlarged=sqrt((1-alpha*kk.*y).^2+(kk.*x).^2)./kk;
turn=(vv*h+.5*aa*h^2).*kk;direct=signs.*(enlarged.*cos(turn/2)-radius);
stable=vpf.CoverDistance(alpha,vv,aa,kk,h,v);coverError=max(abs(direct-stable),[],'all');assert(coverError<1e-12);
monotoneError=0;
for j=1:4
 h=.6;start=[0 0 0 vv(j)];control=[aa(j),atan(v.lw*kk(j))];
 [a,cover]=vpf.ProtectionAlpha([start;start],control,h,v,o);assert(cover.success&&a>=1);
 distance=vv(j)*h+.5*aa(j)*h^2;pp=[distance,kk(j)];finish=cp.ArcPose([0 0 0],pp,abs(distance));
 frame=vpf.Frames([0 0 0;finish],v,a);points=[reshape(frame(:,:,1),[],1),reshape(frame(:,:,2),[],1)];hull=convhull(points);
 tt=linspace(0,h,101)';ss=vv(j)*tt+.5*aa(j)*tt.^2;poses=cp.ArcPose([0 0 0],pp,abs(ss));
 body=vpf.Frames(poses,v,1);xx=body(:,:,1);yy=body(:,:,2);inside=inpolygon(xx,yy,points(hull,1),points(hull,2));
 monotoneError=monotoneError+nnz(~inside);assert(all(inside,'all'));
end
% A local, physically bounded interval with an interior reversal and zero net
% travel. Width-only enlargement cannot cover the extra longitudinal sweep.
start=[0 0 0 -.5];control=[1 0];h=1;finish=vpf.Step(start,control,h,v.lw);
[alpha,reversal]=vpf.ProtectionAlpha([start;finish],control,h,v,o);assert(alpha==1&&reversal.internal_speed_reversals==1);
mid=vpf.Step(start,control,h/2,v.lw);assert(abs(mid(1)+.125)<1e-12);
frame=vpf.Frames([start(1:3);finish(1:3)],v,alpha);body=vpf.Frames(mid(1:3),v,1);
excursion=min(frame(:,:,1),[],'all')-min(body(:,:,1),[],'all');assert(abs(excursion-.125)<1e-12);
[A,B]=vpf.Segments(frame);assert(size(A,1)==12&&isequal(size(A),size(B)));
report=struct('passed',true,'rk4_exact_arc_error',rkError,'signed_radius_formula_error',coverError, ...
 'monotone_cover_outside_samples',monotoneError,'collinear_disjoint_score_is_zero',true, ...
 'interior_reversal_uncovered_excursion_m',excursion);disp(report);
end
