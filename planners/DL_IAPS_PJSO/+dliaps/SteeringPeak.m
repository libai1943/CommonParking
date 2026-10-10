function peak=SteeringPeak(p,speed,lw)
% Exact velocity extrema on each constant spatial steering-slope interval.
% Distance is a monotone cubic and velocity is quadratic in every time cell.
phi=atan(lw*p.kappa);slope=diff(phi)./diff(p.s);peak=0;
for k=1:numel(speed.jerk)
 h=speed.t(k+1)-speed.t(k);s0=speed.s(k);v0=speed.v(k);a0=speed.a(k);j=speed.jerk(k);
 borders=p.s(p.s>s0+1e-10&p.s<speed.s(k+1)-1e-10);time=[0;h];
 for target=borders'
  lo=0;hi=h;
  for iteration=1:48
   mid=(lo+hi)/2;distance=s0+v0*mid+.5*a0*mid^2+j*mid^3/6;
   if distance<target,lo=mid;else,hi=mid;end
  end
  time(end+1)=(lo+hi)/2; %#ok<AGROW>
 end
 time=sort(time);
 for piece=1:numel(time)-1
  limits=time(piece:piece+1);middle=mean(limits);s=s0+v0*middle+.5*a0*middle^2+j*middle^3/6;
  s=max(p.s(1),min(p.s(end),s));
  cellIndex=find(p.s<=s,1,'last');cellIndex=max(1,min(numel(slope),cellIndex));
  candidates=limits;
  if abs(j)>1e-12
   critical=-a0/j;if critical>limits(1)&&critical<limits(2),candidates(end+1)=critical;end
  end
  velocity=v0+a0*candidates+.5*j*candidates.^2;
  peak=max(peak,max(abs(velocity))*abs(slope(cellIndex)));
 end
end
end
