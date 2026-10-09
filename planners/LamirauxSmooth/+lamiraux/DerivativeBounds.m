function [minimumSpeed,maximumSpeed,maximumCurvature]=DerivativeBounds(piece,left,right)
% Outward-padded interval enclosures of P' and P'' on each parameter interval.
left=left(:);right=right(:);middle=(left+right)/2;half=(right-left)/2;v=piece.v;
if strcmp(piece.type,'canonical')
 minimumSpeed=repmat(abs(v),size(left));maximumSpeed=minimumSpeed;maximumCurvature=repmat(abs(piece.a(4)),size(left));return;
end
[A,A1,A2]=canonicalBounds(piece.a,v*middle,v,half);
[B,B1,B2]=canonicalBounds(piece.b,v*(middle-1),v,half);
alpha=[blend(left),blend(right)];one=[1-alpha(:,2),1-alpha(:,1)];
d1=[min(alpha1(left),alpha1(right)),max(alpha1(left),alpha1(right))];inside=left<=.5&right>=.5;d1(inside,2)=1.875;
d2=[min(alpha2(left),alpha2(right)),max(alpha2(left),alpha2(right))];
for critical=[(3-sqrt(3))/6,(3+sqrt(3))/6]
 inside=left<=critical&right>=critical;d2(inside,1)=min(d2(inside,1),alpha2(critical));d2(inside,2)=max(d2(inside,2),alpha2(critical));
end
first=cell(1,2);second=cell(1,2);
for axis=1:2
 delta=subtract(B{axis},A{axis});delta1=subtract(B1{axis},A1{axis});
 first{axis}=multiply(one,A1{axis})+multiply(alpha,B1{axis})+multiply(d1,delta);
 second{axis}=multiply(one,A2{axis})+multiply(alpha,B2{axis})+2*multiply(d1,delta1)+multiply(d2,delta);
end
% Tighten P' with its midpoint value and the enclosed derivative P''.
[~,~,midFirst]=lamiraux.Evaluate(piece,middle);
for axis=1:2
 radius=max(abs(second{axis}),[],2).*half;
 first{axis}=[max(first{axis}(:,1),midFirst(:,axis)-radius),min(first{axis}(:,2),midFirst(:,axis)+radius)];
 pad=1e-12*(1+max(abs(first{axis}),[],2));first{axis}=first{axis}+[-pad pad];
 pad=1e-12*(1+max(abs(second{axis}),[],2));second{axis}=second{axis}+[-pad pad];
end
low=zeros(numel(left),2);high=low;
for axis=1:2,low(:,axis)=max([first{axis}(:,1),-first{axis}(:,2),zeros(numel(left),1)],[],2);high(:,axis)=max(abs(first{axis}),[],2);end
minimumSpeed=hypot(low(:,1),low(:,2));maximumSpeed=hypot(high(:,1),high(:,2));
numerator=subtract(multiply(first{1},second{2}),multiply(first{2},second{1}));maximumCurvature=max(abs(numerator),[],2)./minimumSpeed.^3;
end
function [p,d1,d2]=canonicalBounds(q,s,v,half)
[mid,~,~]=lamiraux.Canonical(q,s,v);angle=q(3)+q(4)*s;radius=abs(q(4)*v)*half;
cs=[max(-1,cos(angle)-radius),min(1,cos(angle)+radius)];sn=[max(-1,sin(angle)-radius),min(1,sin(angle)+radius)];
p={mid(:,1)+abs(v)*[-half half],mid(:,2)+abs(v)*[-half half]};
d1={scale(cs,v),scale(sn,v)};d2={scale(sn,-v^2*q(4)),scale(cs,v^2*q(4))};
end
function z=multiply(a,b)
x=[a(:,1).*b(:,1),a(:,1).*b(:,2),a(:,2).*b(:,1),a(:,2).*b(:,2)];z=[min(x,[],2),max(x,[],2)];
end
function z=subtract(a,b),z=[a(:,1)-b(:,2),a(:,2)-b(:,1)];end
function z=scale(a,b),if b>=0,z=b*a;else,z=b*a(:,[2 1]);end,end
function a=blend(t),a=10*t.^3-15*t.^4+6*t.^5;end
function a=alpha1(t),a=30*t.^2.*(1-t).^2;end
function a=alpha2(t),a=60*t-180*t.^2+120*t.^3;end
