function p=Export(pieces,v,spacing)
% Generic polyline representation with uniform mileage per fixed gear run.
p=struct('s',0,'x',[],'y',[],'theta',[],'phi',[],'gear',[],'cusp_indices',1);offset=0;
for j=1:numel(pieces)
 r=pieces{j};s=linspace(0,r.s(end),max(1,ceil(r.s(end)/spacing))+1)';
 q=interp1(r.s,[r.q atan(v.lw*r.kappa)],s,'linear');
 if j==1,p.x=q(:,1);p.y=q(:,2);p.theta=q(:,3);p.phi=q(:,4);p.s=s;else
  q(:,3)=q(:,3)+2*pi*round((p.theta(end)-q(1,3))/(2*pi));
  p.x=[p.x;q(2:end,1)];p.y=[p.y;q(2:end,2)];p.theta=[p.theta;q(2:end,3)];p.phi=[p.phi;q(2:end,4)];p.s=[p.s;offset+s(2:end)]; %#ok<AGROW>
 end
 p.gear=[p.gear;repmat(r.gear,numel(s)-1,1)];p.cusp_indices(end+1,1)=numel(p.s);offset=p.s(end); %#ok<AGROW>
end
end
