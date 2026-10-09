function Q=Advance(q,length,kappa)
a=length*kappa;h=a/2;r=ones(size(h));nonzero=abs(h)>1e-12;r(nonzero)=sin(h(nonzero))./h(nonzero);
Q=[q(1)+length.*r.*cos(q(3)+h),q(2)+length.*r.*sin(q(3)+h),q(3)+a];
Q(:,3)=atan2(sin(Q(:,3)),cos(Q(:,3)));
end
