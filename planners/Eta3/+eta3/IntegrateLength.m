function lengths=IntegrateLength(piece,breaks)
% Independent Gauss nodes within each supplied parameter interval.
x=[-.9602898564975363;-.7966664774136267;-.525532409916329;-.1834346424956498;.1834346424956498;.525532409916329;.7966664774136267;.9602898564975363];
w=[.1012285362903763;.2223810344533745;.3137066458778873;.362683783378362;.362683783378362;.3137066458778873;.2223810344533745;.1012285362903763];
a=breaks(1:end-1);b=breaks(2:end);a=a(:);b=b(:);u=(a+b)/2+(b-a)/2*x';
[~,speed]=eta3.Evaluate(piece,u(:));speed=reshape(speed,size(u));lengths=(b-a)/2.*(speed*w);
end
