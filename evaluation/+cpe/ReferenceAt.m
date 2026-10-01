function z = ReferenceAt(ref,t,vehicle)
t=min(max(t(:),0),ref.tf);p=ref.data;n=numel(t);
z=struct('t',t,'x',zeros(n,1),'y',zeros(n,1),'theta',zeros(n,1), ...
    'v',zeros(n,1),'phi',zeros(n,1),'a',zeros(n,1),'omega',zeros(n,1));
if strcmp(ref.kind,'path')
    mileage=zeros(n,1);
    for j=1:numel(ref.profiles)
        if j==numel(ref.profiles),mask=t>=ref.times(j);else,mask=t>=ref.times(j)&t<ref.times(j+1);end
        [s,v,a]=cpe.LongitudinalAt(ref.profiles{j},t(mask)-ref.times(j));
        mileage(mask)=ref.milestones(j)+s;z.v(mask)=ref.gear(j)*v;z.a(mask)=ref.gear(j)*a;
    end
    if isfield(p,'geometry')&&strcmp(p.geometry.type,'piecewise_circular')
        [q,kappa]=cp.ArcPose(p.geometry.start,p.geometry.primitives,mileage);
        z.x=q(:,1);z.y=q(:,2);z.theta=q(:,3);z.phi=atan(vehicle.lw*kappa);
    else
        % External sampled paths use the declared poses and steering with
        % linear arc-length interpolation. No hidden smoothing or repair.
        for name={'x','y','theta','phi'}
            field=name{1};values=p.(field);if strcmp(field,'theta'),values=unwrap(values);end
            z.(field)=interp1(p.s,values,mileage,'linear');
        end
    end
else
    for name={'x','y','theta','v','phi','a','omega'}
        field=name{1};values=p.(field);if strcmp(field,'theta'),values=unwrap(values);end
        z.(field)=interp1(p.t,values,t,'linear');
    end
end
z.theta=z.theta+ref.heading_offset;
end
