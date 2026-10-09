function b=Initialize(pose,o,origin)
pose(:,3)=unwrap(pose(:,3));
b=struct('pose',pose,'dt',repmat(o.dt,size(pose,1)-1,1),'cost',Inf,'signature',NaN, ...
 'origin',origin,'solver',struct('success',false));
end
