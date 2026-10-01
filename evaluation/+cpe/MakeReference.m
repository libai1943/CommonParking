function ref = MakeReference(result,c)
ref = struct('kind',result.kind,'data',result.(result.kind));
if strcmp(result.kind,'path') && isfield(ref.data,'geometry')
    angle=ref.data.geometry.start(3);
    ref.data.geometry.start(3)=angle+2*pi*round((ref.data.theta(1)-angle)/(2*pi));
end
if strcmp(result.kind,'trajectory')
    ref.tf=ref.data.t(end);
else
    p=result.path;cuts=p.cusp_indices;profiles=cell(numel(cuts)-1,1);
    times=0;milestones=0;gear=zeros(size(profiles));
    for j=1:numel(profiles)
        gear(j)=p.gear(cuts(j));vmax=c.vehicle.v_forward;
        if gear(j)<0,vmax=c.vehicle.v_reverse;end
        length=p.s(cuts(j+1))-p.s(cuts(j));
        profiles{j}=cpe.LongitudinalProfile(length,vmax,c.vehicle.a_accel,c.vehicle.a_brake);
        times(end+1,1)=times(end)+profiles{j}.tf; %#ok<AGROW>
        milestones(end+1,1)=p.s(cuts(j+1)); %#ok<AGROW>
    end
    ref.profiles=profiles;ref.times=times;ref.milestones=milestones;ref.gear=gear;
    ref.tf=times(end);
end
% Choose one continuous heading branch relative to the true initial pose.
ref.heading_offset=2*pi*round((c.task.theta0-ref.data.theta(1))/(2*pi));
end
