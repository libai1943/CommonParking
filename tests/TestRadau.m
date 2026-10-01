function report = TestRadau()
[tau,D,w]=cp.Radau3();errors=zeros(4,1);
for degree=0:3
    derivative=zeros(3,1);if degree>0,derivative=degree*tau(2:4).^(degree-1);end
    errors(degree+1)=max(abs(D*tau.^degree-derivative));
end
assert(max(errors)<1e-12);
for degree=0:4,assert(abs(w'*tau(2:4).^degree-1/(degree+1))<1e-12);end
% Preserve and expose the paper's vertex-only criterion limitation.
a=[-2 -.2;2 -.2;2 .2;-2 .2];b=[-.2 -2;.2 -2;.2 2;-.2 2];
for q=a',assert(excess(q',b)>.01);end
for q=b',assert(excess(q',a)>.01);end
report=struct('passed',true,'maximum_polynomial_error',max(errors),'crossing_rectangles_pass_area_criterion',true);disp(report);
end
function value=excess(q,p)
a=p-q;b=p([2:end 1],:)-q;value=sum(abs(a(:,1).*b(:,2)-a(:,2).*b(:,1)))/2-polyarea(p(:,1),p(:,2));
end
