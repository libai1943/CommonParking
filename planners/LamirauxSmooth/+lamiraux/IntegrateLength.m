function length=IntegrateLength(piece,left,right)
% Eight-point Gauss-Legendre quadrature; supports a vector of intervals.
x=[-.9602898564975363,-.7966664774136267,-.5255324099163290,-.1834346424956498,.1834346424956498,.5255324099163290,.7966664774136267,.9602898564975363];
w=[.1012285362903763,.2223810344533745,.3137066458778873,.3626837833783620,.3626837833783620,.3137066458778873,.2223810344533745,.1012285362903763];
left=left(:);right=right(:);mid=(left+right)/2;half=(right-left)/2;t=mid+half*x;
[~,speed]=lamiraux.Evaluate(piece,t(:));length=half.*(reshape(speed,[],8)*w');
end
