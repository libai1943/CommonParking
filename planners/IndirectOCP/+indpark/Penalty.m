function [ell,g,H]=Penalty(q,ctx)
% Printed smooth rectangular penalty and its analytic first/second derivatives.
n=size(q,2);ell=zeros(1,n);g=zeros(3,n);H=zeros(3,3,n);
ct=cos(q(3,:));st=sin(q(3,:));epsi=ctx.options.smoothing;
for p=1:size(ctx.points,1)
 a=ctx.points(p,1);b=ctx.points(p,2);dx=a*ct-b*st;dy=a*st+b*ct;
 wx=q(1,:)+dx;wy=q(2,:)+dy;
 for j=1:size(ctx.rectangles,1)
  z=ctx.rectangles(j,:);c=cos(z(3));s=sin(z(3));xx=wx-z(1);yy=wy-z(2);
  [S,S1,S2]=window(c*xx+s*yy,z(4),epsi);[N,N1,N2]=window(-s*xx+c*yy,z(5),epsi);
  d1=S1.*N;d2=S.*N1;h11=S2.*N;h22=S.*N2;h12=S1.*N1;
  J1=[repmat(c,1,n);repmat(s,1,n);-c*dy+s*dx];J2=[repmat(-s,1,n);repmat(c,1,n);s*dy+c*dx];
  ell=ell+S.*N;g=g+J1.*d1+J2.*d2;
  for r=1:3,for k=1:3
   H(r,k,:)=H(r,k,:)+reshape(h11.*J1(r,:).*J1(k,:)+h22.*J2(r,:).*J2(k,:)+h12.*(J1(r,:).*J2(k,:)+J2(r,:).*J1(k,:)),1,1,n);
  end,end
  H(3,3,:)=H(3,3,:)+reshape(d1.*(-c*dx-s*dy)+d2.*(s*dx-c*dy),1,1,n);
 end
end
ell=ctx.options.obstacleWeight*ell;g=ctx.options.obstacleWeight*g;H=ctx.options.obstacleWeight*H;
end
function [q,g,h]=window(x,L,e)
a=x+L/2;b=x-L/2;sa=sqrt(e^2+a.^2);sb=sqrt(e^2+b.^2);
q=.5*(a./sa-b./sb);g=.5*e^2*(sa.^(-3)-sb.^(-3));h=1.5*e^2*(-a.*sa.^(-5)+b.*sb.^(-5));
end
