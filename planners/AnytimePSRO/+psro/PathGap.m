function area=PathGap(path,q)
p=[path.x(:),path.y(:);flipud([q.x(:),q.y(:)])];next=p([2:end 1],:);
area=.5*abs(sum(p(:,1).*next(:,2)-p(:,2).*next(:,1)));
end
