function q=Sample(phases,index,s)
s=s(:);q=zeros(numel(s),4);assigned=false(size(s));
for j=1:size(index.segments,1)
 z=index.segments(j,:);rows=find(s>=z(4)-1e-10&s<=z(4)+z(5)+1e-10);if isempty(rows),continue;end
 target=max(0,min(z(5),s(rows)-z(4)))+z(7);p=phases(z(1));table=index.tables{j};
 if p.n==0,t=target/p.length;else
  t=interp1(table(:,2),table(:,1),target,'pchip');lower=repmat(z(2),size(t));upper=repmat(z(3),size(t));active=true(size(t));
  for iteration=1:60
   ix=find(active);if isempty(ix),break;end
   [~,distance]=sinsteer.Eval(p,t(ix));error=distance-target(ix);done=abs(error)<2e-10;active(ix(done))=false;ix=ix(~done);error=error(~done);if isempty(ix),continue;end
   lower(ix(error<0))=t(ix(error<0));upper(ix(error>0))=t(ix(error>0));
   [~,slope]=sinsteer.Integrand(p,t(ix));candidate=t(ix)-error./max(p.scale*slope,1e-16);
   bad=candidate<=lower(ix)|candidate>=upper(ix)|iteration>12;candidate(bad)=(lower(ix(bad))+upper(ix(bad)))/2;t(ix)=candidate;
  end
  assert(~any(active),'Sinusoid_RTR:Mileage','Mileage inversion did not converge.');
 end
 q(rows,:)=sinsteer.Eval(p,t);assigned(rows)=true;
end
assert(all(assigned),'Sinusoid_RTR:Mileage','Mileage query outside local path.');
end
