function [s,v,a] = LongitudinalAt(p,t)
t=min(max(t(:),0),p.tf); s=zeros(size(t));v=s;a=s;
up=t<p.ta;cruise=t>=p.ta&t<p.ta+p.tc;down=~up&~cruise;
s(up)=0.5*p.accel*t(up).^2;v(up)=p.accel*t(up);a(up)=p.accel;
s(cruise)=p.sa+p.peak*(t(cruise)-p.ta);v(cruise)=p.peak;
remain=p.tf-t(down);s(down)=p.length-0.5*p.brake*remain.^2;
v(down)=p.brake*remain;a(down)=-p.brake;
end
