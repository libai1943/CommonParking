function [f,g]=Penalty(x,a)
% The paper's C1 nonnegative L1 relaxation, equation (64).
f=zeros(size(x));g=f;middle=x>0&x<=a;high=x>a;
y=x(middle);f(middle)=-y.^4/(2*a^3)+y.^3/a^2;g(middle)=-2*y.^3/a^3+3*y.^2/a^2;
f(high)=x(high)-a/2;g(high)=1;
end
