function corridor=Corridor(pose,c,o)
% Section IV-B's direct expansion in fixed directions, here the seed body axes.
v=c.vehicle;R=[cos(pose(3)) -sin(pose(3));sin(pose(3)) cos(pose(3))];polys=parking.PolygonData(c);
local=cellfun(@(p)(p-pose(1:2))*R,polys.vertices,'UniformOutput',false);
box=[-v.lr v.lw+v.lf -v.lb/2 v.lb/2];active=true(1,4);growth=zeros(1,4);
assert(clearBox(box,local),'DFTPAV:SeedCollision','The seed body collides.');
for iteration=1:ceil(o.corridorExtent/o.corridorStep)
 for side=[4 1 3 2]
  if ~active(side),continue;end
  trial=box;direction=1;if ismember(side,[1 3]),direction=-1;end
  trial(side)=trial(side)+direction*o.corridorStep;
  if clearBox(trial,local),box=trial;growth(side)=growth(side)+o.corridorStep;else,active(side)=false;end
  if growth(side)>=o.corridorExtent-1e-9,active(side)=false;end
 end
end
A=[-1 0;1 0;0 -1;0 1]*R';b=[-box(1);box(2);-box(3);box(4)]+A*pose(1:2)';
corridor=struct('A',A,'b',b,'seed',pose,'local_box',box);
end
function safe=clearBox(box,polys)
p=[box(1) box(3);box(2) box(3);box(2) box(4);box(1) box(4)];safe=true;
for k=1:numel(polys)
 z=polys{k};e=z([2:end 1],:)-z;A=[1 0;0 1;e(:,2) -e(:,1)];pa=p*A';pb=z*A';
 if ~any(max(pa,[],1)<min(pb,[],1)-1e-10 | max(pb,[],1)<min(pa,[],1)-1e-10),safe=false;return;end
end
end
