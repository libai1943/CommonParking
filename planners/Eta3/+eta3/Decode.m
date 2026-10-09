function pieces=Decode(z,task,gears)
h=numel(gears);derivatives=reshape(z(1:2*h),2,h)';etas=reshape(z(2*h+1:4*h),2,h)';
interior=reshape(z(4*h+1:end),4,h-1)';
states=[task.x0 task.y0 task.theta0 0;interior;task.xf task.yf task.thetaf 0];
pieces=repmat(eta3.Curve([states(1,:),0],[states(end,:),0],[1 1],1),1,h);
for k=1:h,pieces(k)=eta3.Curve([states(k,:),derivatives(k,1)],[states(k+1,:),derivatives(k,2)],etas(k,:),gears(k));end
end
