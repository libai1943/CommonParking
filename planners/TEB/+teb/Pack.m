function z=Pack(b)
z=[reshape(b.pose(2:end-1,:)',[],1);b.dt];
end
