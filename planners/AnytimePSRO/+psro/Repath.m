function p=Repath(q,v)
% Algorithm 1 takes the optimized pose sequence as the next reference path.
xy=[q.x q.y];keep=[true;vecnorm(diff(xy),2,2)>1e-10];xy=xy(keep,:);theta=unwrap(q.theta(keep));
ds=vecnorm(diff(xy),2,2);gear=sign(sum(diff(xy).*[cos(theta(1:end-1)),sin(theta(1:end-1))],2));
assert(all(gear~=0)&&all(ds>0),'Degenerate reference path.');
kappa=diff(theta)./(gear.*ds);phi=atan(v.lw*[kappa;kappa(end)]);
p=struct('s',[0;cumsum(ds)],'x',xy(:,1),'y',xy(:,2),'theta',theta,'phi',phi,'gear',gear,'cusp_indices',[1;find(diff(gear)~=0)+1;size(xy,1)]);
end
