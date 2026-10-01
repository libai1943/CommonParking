function [q,d,s]=ResampleByGear(q0,pp,ds,extra)
% Equal arc-length spacing within each gear run; include exact cusp events.
if nargin<4,extra=[];end
ends=[0;cumsum(abs(pp(:,1)))];sg=sign(pp(:,1));cuts=[0;ends(find(diff(sg)~=0)+1);ends(end)];
s=[];
for j=1:numel(cuts)-1
 a=cuts(j);b=cuts(j+1);s=[s;linspace(a,b,max(2,ceil((b-a)/ds)+1))']; %#ok<AGROW>
end
s=unique([s;extra(:)]);s=s(s>=0&s<=ends(end));
q=zeros(numel(s),3);base=q0;j=1;
for i=1:numel(s)
 while j<size(pp,1)&&s(i)>ends(j+1)+1e-10
  base=parking.IntegratePrimitive(base,pp(j,1),pp(j,2));j=j+1;
 end
 q(i,:)=parking.IntegratePrimitive(base,sg(j)*(s(i)-ends(j)),pp(j,2));
end
mid=(s(1:end-1)+s(2:end))/2;d=zeros(size(mid));
for j=1:size(pp,1),d(mid>=ends(j)&mid<=ends(j+1))=sg(j);end
end
