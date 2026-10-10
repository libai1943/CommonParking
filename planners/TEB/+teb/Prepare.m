function env=Prepare(c)
env.vehicle=c.vehicle;env.polygons=cell(c.obstacle.num_obs,1);markers=zeros(c.obstacle.num_obs,1);
allxy=[c.task.x0 c.task.y0;c.task.xf c.task.yf];
for j=1:c.obstacle.num_obs
 p=c.obstacle.obs{j};p=[p.x(:) p.y(:)];if norm(p(1,:)-p(end,:))<1e-10,p(end,:)=[];end
 env.polygons{j}=p;markers(j)=mean(p(:,1))+1i*mean(p(:,2));allxy=[allxy;p]; %#ok<AGROW>
end
env.markers=markers;env.origin=mean(allxy);env.scale=max(1,max(max(allxy)-min(allxy)));
z=(markers-complex(env.origin(1),env.origin(2)))/env.scale;R=numel(z);a=ceil(R/2);b=R-a;
lo=(min(allxy)-env.origin)/env.scale;hi=(max(allxy)-env.origin)/env.scale;
BL=complex(lo(1),lo(2));TR=complex(hi(1),hi(2));A=zeros(R,1);
for j=1:R
 if R==1,f0=1;else,f0=a*b*(z(j)-BL)*(z(j)-TR);end
 delta=z(j)-z([1:j-1 j+1:R]);assert(all(abs(delta)>1e-12),'Coincident obstacle markers.');
 A(j)=f0/prod(delta);
end
env.residues=A/max([1;abs(A)]); % Common nonzero scale does not change equivalence.
end
