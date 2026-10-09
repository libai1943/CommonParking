function z=Guess(c,N,o,perturbed)
t=c.task;a=[t.x0;t.y0];b=[t.xf;t.yf];corner=[b(1);a(2)];d1=norm(corner-a);d2=norm(b-corner);phase=linspace(0,1,N);s=phase*(d1+d2);q=zeros(5,N);
for k=1:N
    if s(k)<=d1&&d1>0,q(1:2,k)=a+s(k)/d1*(corner-a);
    elseif d2>0,q(1:2,k)=corner+(s(k)-d1)/d2*(b-corner);
    else,q(1:2,k)=b;end
end
if perturbed
    q(3,:)=.05*sin(2*pi*phase);q(4,:)=.25*sin(2*pi*phase);q(5,:)=.05*sin(pi*phase);
end
q(:,1)=[a;t.theta0;0;0];q(1:4,end)=[b;t.thetaf;0];z=[q(:);o.initialTime];
end
